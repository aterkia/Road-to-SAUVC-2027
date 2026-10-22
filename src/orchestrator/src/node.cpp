#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <aterkia_interfaces/action/navigate_task.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "orchestrator/plan.hpp"

using Task = aterkia_interfaces::action::NavigateTask;
using Handle = rclcpp_action::ClientGoalHandle<Task>;
using Clock = std::chrono::steady_clock;

class Orchestrator : public rclcpp::Node {
 public:
  Orchestrator() : Node("orchestrator") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    const auto vehicle_mode = declare_parameter("vehicle_mode", "mock", fixed);
    hardware_ = vehicle_mode == "hardware";
    if (vehicle_mode != "mock" && !hardware_)
      throw std::invalid_argument("vehicle_mode must be mock or hardware");
    plan_ = declare_parameter<std::vector<std::string>>(
        "tasks", {"PASS_GATE", "TURN", "PASS_GATE"}, fixed);
    timeout_ = declare_parameter("task_timeout", 20.0, fixed);
    health_timeout_ = declare_parameter("health_timeout", 0.5, fixed);
    turn_angle_ = declare_parameter("turn_angle", std::acos(-1.0), fixed);
    if (plan_.empty() || !std::isfinite(timeout_) || timeout_ <= 0 || timeout_ > 120 ||
        !std::isfinite(health_timeout_) || health_timeout_ <= 0 || !std::isfinite(turn_angle_) ||
        std::abs(turn_angle_) < 0.05 || std::abs(turn_angle_) > std::acos(-1.0))
      throw std::invalid_argument("invalid mission parameters");
    orchestrator::validate_prerequisites(plan_);
    orchestrator::require_supported_tasks(plan_, hardware_);
    if (hardware_) {
      start_delay_ = declare_parameter("start_delay_s", 30.0, fixed);
      preparation_timeout_ = declare_parameter("preparation_timeout_s", 8.0, fixed);
      if (!std::isfinite(start_delay_) || start_delay_ <= 0 ||
          !std::isfinite(preparation_timeout_) || preparation_timeout_ <= 0 ||
          plan_.front() != "DIVE_TO_DEPTH")
        throw std::invalid_argument("hardware mission requires countdown and DIVE_TO_DEPTH first");
      ready_navigation_ = create_client<std_srvs::srv::Trigger>("/navigation/ready");
      mode_vehicle_ = create_client<std_srvs::srv::Trigger>("/vehicle/set_mode");
      arm_vehicle_ = create_client<std_srvs::srv::SetBool>("/vehicle/arm");
    }

    status_ = create_publisher<std_msgs::msg::String>(
        "/mission/status", rclcpp::QoS(10).reliable().transient_local());
    navigation_ = rclcpp_action::create_client<Task>(this, "/navigation/task");
    abort_vehicle_ = create_client<std_srvs::srv::Trigger>("/safety/abort");

    health_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/vehicle/health",
        rclcpp::QoS(1).reliable(),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
          armed_ = false;
          preflight_ = false;
          sensors_ok_ = false;
          if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000)
            return;
          const auto age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (age < -0.05 || age > health_timeout_ || stamp <= health_stamp_)
            return;
          health_stamp_ = stamp;
          for (const auto &s : msg->status)
            if (s.name == "helm") {
              armed_ = s.level == 0 && s.message == "ARMED";
              bool imu = false, depth = false, configured = false;
              for (const auto &field : s.values) {
                if (field.key == "preflight_ready") preflight_ = field.value == "true";
                if (field.key == "imu_fresh") imu = field.value == "true";
                if (field.key == "depth_available") depth = field.value == "true";
                if (field.key == "fcu_control_config_verified") configured = field.value == "true";
              }
              sensors_ok_ = imu && depth && configured;
            }
          last_health_ = Clock::now();
        });

    // Start only with fresh armed health and available navigation/stop endpoints.
    start_ = create_service<std_srvs::srv::Trigger>("/mission/start",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response) {
          if (hardware_) {
            response->message = "Autonomous hardware starts only after its onboard countdown";
            return;
          }
          if (state_ == "RUNNING" || pending_ || goal_ || !healthy() ||
              !navigation_->action_server_is_ready() || !abort_vehicle_->service_is_ready()) {
            response->message =
                "preflight failed: arm, telemetry, action server or previous task still active";
            return;
          }
          index_ = 0;
          state_ = "RUNNING";
          publish("RUNNING");
          dispatch();
          response->success = true;
        });

    abort_ = create_service<std_srvs::srv::Trigger>("/mission/abort",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response) {
          stop("ABORTED", "operator request");
          response->success = true;
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      if (hardware_tick())
        return;
      if (state_ != "RUNNING")
        return;
      if (!healthy()) {
        stop("FAILED", "vehicle health unavailable or disarmed");
        return;
      }
      if (std::chrono::duration<double>(Clock::now() - sent_).count() > timeout_ + 1.0)
        stop("FAILED", "action response/result timeout");
    });
    if (hardware_) {
      state_ = "COUNTDOWN";
      countdown_started_ = Clock::now();
      publish("COUNTDOWN: " + std::to_string(start_delay_) + " s; disarmed preparation");
    } else
      publish("IDLE");
    for (size_t i = 0; i < plan_.size(); ++i)
      RCLCPP_INFO(get_logger(), "Plan %zu: %s", i + 1, plan_[i].c_str());
  }

 private:
  bool health_fresh() const {
    const double age = (now().nanoseconds() - health_stamp_) / 1e9;
    return health_stamp_ > 0 && age >= -0.05 && age < health_timeout_ &&
        std::chrono::duration<double>(Clock::now() - last_health_).count() < health_timeout_;
  }

  bool hardware_tick() {
    if (!hardware_)
      return false;
    if (state_ == "COUNTDOWN") {
      const double elapsed = std::chrono::duration<double>(Clock::now() - countdown_started_).count();
      if (elapsed >= start_delay_) {
        if (!health_fresh() || !preflight_ || !navigation_->action_server_is_ready() ||
            !abort_vehicle_->service_is_ready() || !ready_navigation_->service_is_ready() ||
            !mode_vehicle_->service_is_ready() || !arm_vehicle_->service_is_ready()) {
          stop("FAILED", "countdown ended but preflight was not ready");
          return true;
        }
        state_ = "PREPARING";
        preparation_started_ = Clock::now();
        publish("PREPARING: guidance readiness");
        const auto generation = ++generation_;
        ready_navigation_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this, generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture reply) {
              if (!preparing(generation)) return;
              if (!reply.get()->success)
                stop("FAILED", "guidance preflight: " + reply.get()->message);
              else
                prepare_mode(generation);
            });
      } else {
        const auto remaining = static_cast<int>(std::ceil(start_delay_ - elapsed));
        if (remaining != last_countdown_) {
          last_countdown_ = remaining;
          publish("COUNTDOWN: " + std::to_string(remaining) + " s remaining");
        }
      }
      return true;
    }
    if (state_ == "PREPARING") {
      if (!health_fresh() || !sensors_ok_ ||
          std::chrono::duration<double>(Clock::now() - preparation_started_).count() > preparation_timeout_)
        stop("FAILED", "preparation timed out or telemetry became invalid");
      else if (arm_confirmed_ && healthy()) {
        index_ = 0;
        state_ = "RUNNING";
        publish("RUNNING");
        dispatch();
      }
      return true;
    }
    if (state_ == "DISARMING") {
      if (std::chrono::duration<double>(Clock::now() - stopping_started_).count() > preparation_timeout_) {
        state_ = "FAILED";
        publish("FAILED: disarm unconfirmed; requested terminal state " + terminal_state_);
      }
      return true;
    }
    return state_ != "RUNNING";
  }

  bool preparing(uint64_t generation) const {
    return generation == generation_ && state_ == "PREPARING";
  }

  void prepare_mode(uint64_t generation) {
    publish("PREPARING: ALT_HOLD");
    mode_vehicle_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this, generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture reply) {
          if (!preparing(generation)) return;
          if (!reply.get()->success) {
            stop("FAILED", "mode: " + reply.get()->message);
            return;
          }
          publish("PREPARING: arming");
          auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
          request->data = true;
          arm_vehicle_->async_send_request(request,
              [this, generation](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {
                if (!preparing(generation)) return;
                if (!result.get()->success)
                  stop("FAILED", "arming: " + result.get()->message);
                else
                  arm_confirmed_ = true;
              });
        });
  }

  bool healthy() const {
    double age = (now().nanoseconds() - health_stamp_) / 1e9;
    return armed_ && age >= -0.05 && age < health_timeout_ &&
        std::chrono::duration<double>(Clock::now() - last_health_).count() < health_timeout_;
  }

  void publish(const std::string &text) {
    std_msgs::msg::String msg;
    msg.data = text;
    status_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void disarm() {
    if (hardware_) {
      terminal_state_ = state_;
      state_ = "DISARMING";
      stopping_started_ = Clock::now();
      const auto generation = ++generation_;
      publish("DISARMING: " + terminal_state_);
      if (!arm_vehicle_->service_is_ready()) {
        state_ = "FAILED";
        publish("FAILED: disarm service unavailable; requested " + terminal_state_);
        return;
      }
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = false;
      arm_vehicle_->async_send_request(request,
          [this, generation](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture reply) {
            if (generation != generation_ || state_ != "DISARMING") return;
            state_ = reply.get()->success ? terminal_state_ : "FAILED";
            publish(state_ + (reply.get()->success ? ": disarm confirmed" :
                ": disarm unconfirmed: " + reply.get()->message));
          });
      return;
    }
    // Use the independent helm stop path even if navigation cannot respond.
    if (abort_vehicle_->service_is_ready())
      abort_vehicle_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  void stop(const std::string &state, const std::string &reason) {
    if (hardware_ && (state_ == "DISARMING" || state_ == "COMPLETED" ||
            state_ == "FAILED" || state_ == "ABORTED"))
      return;
    state_ = state;
    publish(state + ": " + reason);
    if (goal_)
      navigation_->async_cancel_goal(goal_);
    disarm();
  }

  void dispatch() {
    if (index_ == plan_.size()) {
      state_ = "COMPLETED";
      if (!hardware_)
        publish(state_);
      disarm();
      return;
    }
    publish("RUNNING: " + std::to_string(index_ + 1) + "/" + std::to_string(plan_.size()) + " " +
        plan_[index_]);
    Task::Goal task;
    task.task = plan_[index_] == "NAVIGATION" ? "PASS_GATE" : plan_[index_];
    task.timeout = timeout_;
    task.turn_radians = turn_angle_;
    pending_ = true;
    sent_ = Clock::now();

    // A goal accepted after an operator abort must still be canceled.
    rclcpp_action::Client<Task>::SendGoalOptions options;
    options.goal_response_callback = [this](Handle::SharedPtr handle) {
      pending_ = false;
      goal_ = handle;
      if (!handle) {
        if (state_ == "RUNNING")
          stop("FAILED", "navigation rejected task");
      } else if (state_ != "RUNNING")
        navigation_->async_cancel_goal(handle);
    };

    // Advance the plan only after a confirmed successful Action result.
    options.result_callback = [this](const Handle::WrappedResult &result) {
      goal_.reset();
      pending_ = false;
      if (state_ != "RUNNING")
        return;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result->status == "COMPLETED") {
        publish("TASK_COMPLETED: " + std::to_string(index_ + 1) + " " + plan_[index_]);
        ++index_;
        dispatch();
      } else
        stop(result.result && result.result->status == "ABORTED" ? "ABORTED" : "FAILED",
            result.result ? result.result->reason : "missing navigation result");
    };
    navigation_->async_send_goal(task, options);
  }

  std::vector<std::string> plan_;
  size_t index_{0};
  std::string state_{"IDLE"};
  double timeout_, health_timeout_, turn_angle_;
  bool armed_{false}, pending_{false};
  bool hardware_{false}, preflight_{false}, sensors_ok_{false}, arm_confirmed_{false};
  double start_delay_{30}, preparation_timeout_{8};
  int last_countdown_{-1};
  uint64_t generation_{0};
  std::string terminal_state_;
  Clock::time_point countdown_started_, preparation_started_, stopping_started_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready_navigation_, mode_vehicle_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_vehicle_;
  Clock::time_point last_health_, sent_;
  int64_t health_stamp_{0};
  Handle::SharedPtr goal_;
  rclcpp_action::Client<Task>::SharedPtr navigation_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr abort_vehicle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_, abort_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Orchestrator>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("orchestrator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
