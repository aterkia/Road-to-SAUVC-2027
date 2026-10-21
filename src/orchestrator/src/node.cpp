#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <aterkia_interfaces/action/navigate_task.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "orchestrator/plan.hpp"

using Task = aterkia_interfaces::action::NavigateTask;
using Handle = rclcpp_action::ClientGoalHandle<Task>;
using Clock = std::chrono::steady_clock;
class Orchestrator : public rclcpp::Node {
 public:
  Orchestrator() : Node("orchestrator") {
    rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only = true;
    plan_ = declare_parameter<std::vector<std::string>>("tasks", {"PASS_GATE", "TURN", "PASS_GATE"}, fixed);
    timeout_ = declare_parameter("task_timeout", 20.0, fixed);
    health_timeout_ = declare_parameter("health_timeout", 0.5, fixed);
    turn_angle_ = declare_parameter("turn_angle", std::acos(-1.0), fixed);
    if (plan_.empty() || !std::isfinite(timeout_) || timeout_ <= 0 || timeout_ > 120 ||
        !std::isfinite(health_timeout_) || health_timeout_ <= 0 ||
        !std::isfinite(turn_angle_) || std::abs(turn_angle_) < 0.05 || std::abs(turn_angle_) > std::acos(-1.0))
      throw std::invalid_argument("invalid mission parameters");
    orchestrator::validate_prerequisites(plan_);
    orchestrator::require_supported_tasks(plan_);
    status_ = create_publisher<std_msgs::msg::String>("/mission/status", rclcpp::QoS(10).reliable().transient_local());
    navigation_ = rclcpp_action::create_client<Task>(this, "/navigation/task");
    abort_vehicle_ = create_client<std_srvs::srv::Trigger>("/safety/abort");
    health_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/vehicle/health", rclcpp::QoS(1).reliable(),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        armed_ = false;
        if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000) return;
        const auto age = (now()-rclcpp::Time(msg->header.stamp)).seconds();
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (age < -0.05 || age > health_timeout_ || stamp <= health_stamp_) return;
        health_stamp_ = stamp;
        for (const auto &s : msg->status) if (s.name == "helm") armed_ = s.level == 0 && s.message == "ARMED";
        last_health_ = Clock::now();
      });
    start_ = create_service<std_srvs::srv::Trigger>("/mission/start",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (state_ == "RUNNING" || pending_ || goal_ || !healthy() || !navigation_->action_server_is_ready() ||
            !abort_vehicle_->service_is_ready()) {
          response->message = "preflight failed: arm, telemetry, action server or previous task still active";
          return;
        }
        index_ = 0; state_ = "RUNNING"; publish("RUNNING"); dispatch();
        response->success = true;
      });
    abort_ = create_service<std_srvs::srv::Trigger>("/mission/abort",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        stop("ABORTED", "operator request"); response->success = true;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      if (state_ != "RUNNING") return;
      if (!healthy()) { stop("FAILED", "vehicle health unavailable or disarmed"); return; }
      if (std::chrono::duration<double>(Clock::now()-sent_).count() > timeout_ + 1.0)
        stop("FAILED", "action response/result timeout");
    });
    publish("IDLE");
    for (size_t i = 0; i < plan_.size(); ++i)
      RCLCPP_INFO(get_logger(), "Plan %zu: %s", i+1, plan_[i].c_str());
  }
 private:
  bool healthy() const {
    double age = (now().nanoseconds()-health_stamp_) / 1e9;
    return armed_ && age >= -0.05 && age < health_timeout_ &&
      std::chrono::duration<double>(Clock::now()-last_health_).count() < health_timeout_;
  }
  void publish(const std::string &text) {
    std_msgs::msg::String msg; msg.data = text; status_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }
  void disarm() {
    if (abort_vehicle_->service_is_ready())
      abort_vehicle_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }
  void stop(const std::string &state, const std::string &reason) {
    state_ = state; publish(state + ": " + reason);
    if (goal_) navigation_->async_cancel_goal(goal_);
    disarm();
  }
  void dispatch() {
    if (index_ == plan_.size()) {
      state_ = "COMPLETED"; publish(state_); disarm(); return;
    }
    publish("RUNNING: " + std::to_string(index_+1) + "/" + std::to_string(plan_.size()) + " " + plan_[index_]);
    Task::Goal task; task.task = plan_[index_] == "NAVIGATION" ? "PASS_GATE" : plan_[index_];
    task.timeout = timeout_; task.turn_radians = turn_angle_;
    pending_ = true; sent_ = Clock::now();
    rclcpp_action::Client<Task>::SendGoalOptions options;
    options.goal_response_callback = [this](Handle::SharedPtr handle) {
      pending_ = false; goal_ = handle;
      if (!handle) {
        if (state_ == "RUNNING") stop("FAILED", "navigation rejected task");
      } else if (state_ != "RUNNING") navigation_->async_cancel_goal(handle);
    };
    options.result_callback = [this](const Handle::WrappedResult &result) {
      goal_.reset(); pending_ = false;
      if (state_ != "RUNNING") return;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->status == "COMPLETED") {
        publish("TASK_COMPLETED: " + std::to_string(index_+1) + " " + plan_[index_]);
        ++index_; dispatch();
      } else stop(result.result && result.result->status == "ABORTED" ? "ABORTED" : "FAILED",
                  result.result ? result.result->reason : "missing navigation result");
    };
    navigation_->async_send_goal(task, options);
  }
  std::vector<std::string> plan_;
  size_t index_{0};
  std::string state_{"IDLE"};
  double timeout_, health_timeout_, turn_angle_;
  bool armed_{false}, pending_{false};
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
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<Orchestrator>()); rclcpp::shutdown();
}
