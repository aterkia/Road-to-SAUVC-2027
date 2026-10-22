#include <chrono>
#include <cmath>
#include <optional>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <aterkia_interfaces/msg/depth_stamped.hpp>
#include "vehicle_control.hpp"

using Steady = std::chrono::steady_clock;

// Vehicle telemetry and optional onboard control; bench profiles remain separate.
class MavrosMonitor : public rclcpp::Node {
 public:
  MavrosMonitor() : Node("helm") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    const bool control_enabled = declare_parameter("control_enabled", false, fixed);
    firmware_ = declare_parameter("firmware_family", "ardusub", fixed);
    version_ = declare_parameter("firmware_version", "unverified", fixed);
    mode_ = declare_parameter("control_mode", "", fixed);
    frame_ = declare_parameter("imu_frame", "base_link", fixed);
    state_timeout_ = positive("state_timeout", 3.0, fixed);
    sensor_timeout_ = positive("telemetry_timeout", 0.5, fixed);
    arming_enabled_ = declare_parameter("arming_enabled", false, fixed);
    arm_timeout_ = positive("arming_timeout", 6.0, fixed);
    if (arming_enabled_)
      arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    depth_enabled_ = declare_parameter("pressure_is_depth_sensor", false, fixed);
    surface_pressure_ = declare_parameter("surface_pressure_pa", 0.0, fixed);
    density_ = declare_parameter("water_density_kg_m3", 0.0, fixed);
    gravity_ = positive("gravity_m_s2", 9.80665, fixed);
    if (firmware_.empty() || version_.empty() || frame_.empty() ||
        (depth_enabled_ &&
            (!std::isfinite(surface_pressure_) || surface_pressure_ <= 0 ||
                !std::isfinite(density_) || density_ <= 0)))
      throw std::invalid_argument(
          "invalid firmware/frame or uncalibrated pressure-to-depth configuration");
    if (control_enabled) {
      if (!arming_enabled_ || firmware_ != "ardusub" || version_ == "unverified" ||
          mode_ != "ALT_HOLD" || !depth_enabled_ || frame_ != "base_link")
        throw std::invalid_argument(
            "onboard control requires ArduSub version, ALT_HOLD, calibrated depth and arming_enabled");
      control_ = std::make_unique<VehicleControl>(*this);
      mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    }

    health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/vehicle/health", rclcpp::QoS(1).reliable());
    state_pub_ =
        create_publisher<mavros_msgs::msg::State>("/vehicle/state", rclcpp::QoS(1).reliable());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/vehicle/imu", rclcpp::SensorDataQoS());
    pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
        "/vehicle/pressure", rclcpp::SensorDataQoS());
    heading_pub_ =
        create_publisher<std_msgs::msg::Float64>("/vehicle/heading", rclcpp::SensorDataQoS());
    depth_pub_ =
        create_publisher<std_msgs::msg::Float64>("/vehicle/depth", rclcpp::SensorDataQoS());
    depth_stamped_pub_ = create_publisher<aterkia_interfaces::msg::DepthStamped>(
        "/vehicle/depth_stamped", rclcpp::SensorDataQoS());

    const auto state_topic = declare_parameter("state_topic", "/mavros/state", fixed);
    const auto imu_topic = declare_parameter("imu_topic", "/mavros/imu/data", fixed);
    const auto pressure_topic =
        declare_parameter("pressure_topic", "/mavros/imu/static_pressure", fixed);

    state_sub_ = create_subscription<mavros_msgs::msg::State>(state_topic,
        rclcpp::QoS(10).reliable(), [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
          if (!accept(msg->header.stamp, state_sample_, state_timeout_))
            return;
          state_ = *msg;
          state_pub_->publish(*msg);
          if (!msg->connected) {
            imu_sample_ = Sample{};
            pressure_sample_ = Sample{};
          }
        });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          const auto &q = msg->orientation;
          const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
          if (!connected() || msg->header.frame_id != frame_ ||
              msg->orientation_covariance[0] < 0 || !std::isfinite(norm) ||
              std::abs(norm - 1.0) > 0.01)
            return;
          for (double v : msg->orientation_covariance)
            if (!std::isfinite(v))
              return;
          const double values[] = {msg->angular_velocity.x, msg->angular_velocity.y,
              msg->angular_velocity.z, msg->linear_acceleration.x, msg->linear_acceleration.y,
              msg->linear_acceleration.z};
          for (double v : values)
            if (!std::isfinite(v))
              return;
          if (!accept(msg->header.stamp, imu_sample_, sensor_timeout_))
            return;
          imu_pub_->publish(*msg);
          // MAVROS already converts NED/FRD to ENU/FLU. Do not convert it again.
          std_msgs::msg::Float64 heading;
          heading.data = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
          heading_pub_->publish(heading);
        });

    pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(pressure_topic,
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {
          if (!connected() || msg->header.frame_id != frame_ ||
              !std::isfinite(msg->fluid_pressure) || msg->fluid_pressure <= 0 ||
              !std::isfinite(msg->variance) || msg->variance < 0 ||
              !accept(msg->header.stamp, pressure_sample_, sensor_timeout_))
            return;
          pressure_pub_->publish(*msg);
          if (depth_enabled_) {
            std_msgs::msg::Float64 depth;
            depth.data = (msg->fluid_pressure - surface_pressure_) / (density_ * gravity_);
            if (std::isfinite(depth.data)) {
              depth_pub_->publish(depth);
              aterkia_interfaces::msg::DepthStamped stamped;
              stamped.header = msg->header;
              stamped.depth_m = depth.data;
              depth_stamped_pub_->publish(stamped);
            }
          }
        });

    // Defer the response so telemetry continues while waiting for the FCU.
    arm_ = create_service<std_srvs::srv::SetBool>("/vehicle/arm",
        [this](std::shared_ptr<rmw_request_id_t> header,
            std_srvs::srv::SetBool::Request::SharedPtr req) {
          request_arm(header, req->data);
        });

    mode_service_ = create_service<std_srvs::srv::Trigger>("/vehicle/set_mode",
        [this](std::shared_ptr<rmw_request_id_t> header,
            std_srvs::srv::Trigger::Request::SharedPtr) {
          request_mode(header);
        });

    abort_ = create_service<std_srvs::srv::Trigger>("/safety/abort",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr res) {
          res->success = false;
          res->message =
              "Motion abort is unavailable; use independent vehicle safety or explicit disarm";
          if (control_) {
            stop_control("ABORTED");
            res->success = true;
            res->message = "Command session cancelled; disarm requested, check FCU state for confirmation";
          }
        });

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(1).reliable(), [this](geometry_msgs::msg::Twist::ConstSharedPtr) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000, "READ_ONLY: motion command rejected");
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      check_arm();
      check_mode();
      if (control_) {
        control_->tick(connected(), sensors_fresh(), state_.mode == mode_, state_.armed);
        if (session_active_ && !control_->authorized())
          stop_control(control_->reason());
        if (!state_.armed)
          disarm_attempted_ = false;
        if (must_disarm_ && connected() && state_.armed && !pending_arm_ && !disarm_attempted_) {
          disarm_attempted_ = true;
          request_arm(nullptr, false);
        }
      }
      health();
    });
    RCLCPP_INFO(
        get_logger(), "%s",
        control_ ? "Onboard ALT_HOLD control enabled; awaiting mission preparation" :
        arming_enabled_ ? "ARMING_ONLY: explicit arm/disarm enabled" : "READ_ONLY MAVROS telemetry");
  }

 private:
  struct Sample {
    int64_t stamp{0};
    Steady::time_point received{};
  };

  struct PendingArm {
    std::shared_ptr<rmw_request_id_t> header;
    bool target;
    Steady::time_point started;
    int64_t request_id{0};
    bool accepted{false};
    uint64_t generation{0};
  };

  struct PendingMode {
    std::shared_ptr<rmw_request_id_t> header;
    Steady::time_point started;
    int64_t request_id{0};
    bool sent{false};
    uint64_t generation{0};
  };

  bool sensors_fresh() const {
    return fresh(imu_sample_, sensor_timeout_) && depth_enabled_ &&
        fresh(pressure_sample_, sensor_timeout_);
  }

  void stop_control(const std::string &reason) {
    control_->stop(reason);
    prepared_ = false;
    session_active_ = false;
    must_disarm_ = true;
    if (pending_mode_)
      finish_mode(false, "Mode preparation cancelled");
    if (pending_arm_ && pending_arm_->target)
      finish_arm(false, "Arming authorization cancelled; disarm required");
  }

  void mode_response(const std::shared_ptr<rmw_request_id_t> &header,
      bool success, const std::string &message) {
    std_srvs::srv::Trigger::Response response;
    response.success = success;
    response.message = message;
    mode_service_->send_response(*header, response);
  }

  void finish_mode(bool success, const std::string &message) {
    auto header = pending_mode_->header;
    prepared_ = success;
    mode_client_->remove_pending_request(pending_mode_->request_id);
    pending_mode_.reset();
    mode_response(header, success, message);
  }

  void request_mode(const std::shared_ptr<rmw_request_id_t> &header) {
    if (!control_ || !connected() || !sensors_fresh() || !control_->configured() ||
        state_.armed || pending_mode_ || pending_arm_) {
      mode_response(header, false,
          "Preparation requires verified control configuration, fresh sensors, disarmed FCU and no pending request");
      return;
    }
    if (!mode_client_->service_is_ready()) {
      mode_response(header, false, "MAVROS mode service is unavailable");
      return;
    }
    control_->stop("PREPARING");
    prepared_ = false;
    must_disarm_ = false;
    pending_mode_ = PendingMode{header, Steady::now()};
    pending_mode_->generation = control_->generation();
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode_;
    auto future = mode_client_->async_send_request(request,
        [this, header](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture reply) {
          if (!pending_mode_ || pending_mode_->header != header)
            return;
          if (!reply.get()->mode_sent)
            finish_mode(false, "MAVROS rejected mode request");
          else
            pending_mode_->sent = true;
        });
    pending_mode_->request_id = future.request_id;
  }

  void check_mode() {
    if (!pending_mode_)
      return;
    if (!connected() || !sensors_fresh() || !control_->configured() || state_.armed ||
        pending_mode_->generation != control_->generation())
      finish_mode(false, "Mode preparation interrupted");
    else if (std::chrono::duration<double>(Steady::now() - pending_mode_->started).count() >= arm_timeout_)
      finish_mode(false, "Timeout: mode change unconfirmed; inspect FCU state");
    else if (pending_mode_->sent && state_.mode == mode_ && state_sample_.received > pending_mode_->started)
      finish_mode(true, "ALT_HOLD confirmed by FCU state");
  }

  void arm_response(const std::shared_ptr<rmw_request_id_t> &header,
      bool success, const std::string &message) {
    std_srvs::srv::SetBool::Response response;
    response.success = success;
    response.message = message;
    if (header)
      arm_->send_response(*header, response);
    else
      RCLCPP_WARN(get_logger(), "Safety disarm: %s", message.c_str());
  }

  void finish_arm(bool success, const std::string &message) {
    if (control_ && pending_arm_->target) {
      if (success && prepared_ && pending_arm_->generation == control_->generation() &&
          sensors_fresh() && state_.mode == mode_ && control_->configured()) {
        control_->authorize();
        session_active_ = true;
        prepared_ = false;
      } else {
        success = false;
        control_->stop("ARMING_FAILED");
        prepared_ = false;
        must_disarm_ = true;
      }
    }
    auto header = pending_arm_->header;
    const bool disarming = !pending_arm_->target;
    arming_client_->remove_pending_request(pending_arm_->request_id);
    pending_arm_.reset();
    arm_response(header, success, message);
    if (disarming) {
      for (const auto &waiter : disarm_waiters_)
        arm_response(waiter, success, message);
      disarm_waiters_.clear();
    }
  }

  void request_arm(const std::shared_ptr<rmw_request_id_t> &header, bool target) {
    if (!arming_enabled_) {
      arm_response(header, false, "READ_ONLY: launch arming.launch.py to enable explicit arm/disarm");
      return;
    }
    const bool cancelled_arm = !target && control_ && pending_arm_ && pending_arm_->target;
    if (!target && control_ && header)
      stop_control("DISARM_REQUESTED");
    // Mission shutdown may join a safety disarm already awaiting FCU confirmation.
    if (!target && control_ && pending_arm_ && !pending_arm_->target && header) {
      disarm_waiters_.push_back(header);
      return;
    }
    if (pending_arm_ || pending_mode_) {
      arm_response(header, false, "Another arm/disarm request is pending");
      return;
    }
    if (target && control_ && (!prepared_ || state_.armed || state_.mode != mode_ ||
        !sensors_fresh() || !control_->configured() || must_disarm_)) {
      arm_response(header, false, "Mission preparation with fresh sensors and ALT_HOLD is required before arming");
      return;
    }
    if (!connected()) {
      arm_response(header, false, "DISCONNECTED: fresh FCU state is required");
      return;
    }
    if (state_.armed == target) {
      if (cancelled_arm) {
        arm_response(header, false, "Previous arming outcome unconfirmed; command session cancelled and safety disarm pending");
        return;
      }
      arm_response(header, true, target ? "Already armed" : "Already disarmed");
      return;
    }
    // A stale IMU must not prevent an explicit disarm request.
    if (target && !fresh(imu_sample_, sensor_timeout_)) {
      arm_response(header, false, "TELEMETRY_STALE: fresh IMU is required to arm");
      return;
    }
    if (!arming_client_->service_is_ready()) {
      arm_response(header, false, "MAVROS arming service is unavailable");
      return;
    }
    pending_arm_ = PendingArm{header, target, Steady::now()};
    pending_arm_->generation = control_ ? control_->generation() : 0;
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = target;
    auto future = arming_client_->async_send_request(request,
        [this, header](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture reply) {
          // Ignore replies arriving after a timeout or disconnect. Never retry automatically.
          if (!pending_arm_ || pending_arm_->header != header)
            return;
          const auto result = reply.get();
          if (!result->success || result->result != 0) {
            finish_arm(false, "FCU rejected arm/disarm; MAV_RESULT=" + std::to_string(result->result));
            return;
          }
          pending_arm_->accepted = true;
        });
    pending_arm_->request_id = future.request_id;
  }

  void check_arm() {
    if (!pending_arm_)
      return;
    if (control_ && pending_arm_->target &&
        (!sensors_fresh() || state_.mode != mode_ || !control_->configured() ||
            pending_arm_->generation != control_->generation())) {
      finish_arm(false, "Arming preparation invalidated; outcome unconfirmed");
      return;
    }
    if (!connected()) {
      finish_arm(false, "DISCONNECTED: arm/disarm outcome is unconfirmed; inspect FCU state");
    } else if (std::chrono::duration<double>(Steady::now() - pending_arm_->started).count() >=
        arm_timeout_) {
      finish_arm(false, "Timeout: arm/disarm outcome is unconfirmed; inspect FCU state");
    } else if (pending_arm_->accepted && state_.armed == pending_arm_->target &&
        state_sample_.received > pending_arm_->started) {
      finish_arm(true, state_.armed ? "Armed: confirmed by FCU state" : "Disarmed: confirmed by FCU state");
    }
  }

  double positive(
      const char *name, double value, const rcl_interfaces::msg::ParameterDescriptor &fixed) {
    const double v = declare_parameter(name, value, fixed);
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument(name);
    return v;
  }

  bool accept(const builtin_interfaces::msg::Time &stamp, Sample &sample, double timeout) {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000)
      return false;

    // Repeated source timestamps must not keep a frozen sensor fresh.
    const int64_t ns = rclcpp::Time(stamp).nanoseconds();
    const double age = (now().nanoseconds() - ns) / 1e9;
    if (ns <= sample.stamp || age < -0.05 || age > timeout)
      return false;
    sample = {ns, Steady::now()};
    return true;
  }

  bool fresh(const Sample &sample, double timeout) const {
    // Check source age and monotonic receipt time to detect stale streams.
    const double age = (now().nanoseconds() - sample.stamp) / 1e9;
    return sample.stamp > 0 && age >= -0.05 && age < timeout &&
        std::chrono::duration<double>(Steady::now() - sample.received).count() < timeout;
  }

  bool connected() const {
    return state_.connected && fresh(state_sample_, state_timeout_);
  }

  void health() {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "helm";
    status.hardware_id = firmware_ + ":" + version_;
    const bool imu_ok = connected() && fresh(imu_sample_, sensor_timeout_);
    const bool pressure_ok = connected() && fresh(pressure_sample_, sensor_timeout_);
    status.level = connected() ? 1 : 2;
    status.message =
        !connected() ? "DISCONNECTED" :
        !imu_ok || (depth_enabled_ && !pressure_ok) ? "TELEMETRY_STALE" :
        control_ ? (control_->authorized() ? "ARMED" : control_->reason()) :
        arming_enabled_ ? "ARMING_ONLY" : "READ_ONLY";
    if (control_ && control_->authorized() && connected() && sensors_fresh() && state_.mode == mode_)
      status.level = 0;
    auto field = [&status](const std::string &key, const std::string &value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };
    field("control_enabled", control_ ? "true" : "false");
    field("motion_authorized", control_ && control_->authorized() ? "true" : "false");
    field("preflight_ready", control_ && connected() && sensors_fresh() &&
        control_->configured() && !state_.armed && !pending_arm_ && !pending_mode_ ? "true" : "false");
    field("fcu_control_config_verified", control_ && control_->configured() ? "true" : "false");
    if (control_)
      field("fcu_control_config_error", control_->configuration_error());
    field("arming_enabled", arming_enabled_ ? "true" : "false");
    field("arming_service_ready", arming_client_ && arming_client_->service_is_ready() ? "true" : "false");
    field("configured_mode", mode_);
    field("observed_mode", connected() ? state_.mode : "unknown");
    field("observed_armed", connected() ? (state_.armed ? "true" : "false") : "unknown");
    field("manual_input", connected() ? (state_.manual_input ? "true" : "false") : "unknown");
    field("imu_fresh", imu_ok ? "true" : "false");
    field("pressure_fresh", pressure_ok ? "true" : "false");
    field("depth_available", depth_enabled_ && pressure_ok ? "true" : "false");
    msg.status.push_back(status);
    health_pub_->publish(msg);
    if (status.message != last_status_) {
      RCLCPP_WARN(get_logger(), "%s", status.message.c_str());
      last_status_ = status.message;
    }
  }

  std::string firmware_, version_, mode_, frame_, last_status_;
  double state_timeout_, sensor_timeout_, surface_pressure_, density_, gravity_, arm_timeout_;
  bool depth_enabled_, arming_enabled_;
  std::optional<PendingArm> pending_arm_;
  std::vector<std::shared_ptr<rmw_request_id_t>> disarm_waiters_;
  std::optional<PendingMode> pending_mode_;
  std::unique_ptr<VehicleControl> control_;
  bool prepared_{false}, session_active_{false}, must_disarm_{false}, disarm_attempted_{false};
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  Sample state_sample_, imu_sample_, pressure_sample_;
  mavros_msgs::msg::State state_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_, depth_pub_;
  rclcpp::Publisher<aterkia_interfaces::msg::DepthStamped>::SharedPtr depth_stamped_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mode_service_, abort_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MavrosMonitor>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("helm"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
