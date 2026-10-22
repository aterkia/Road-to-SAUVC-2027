#pragma once

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <aterkia_interfaces/msg/vehicle_command.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

// Transport for onboard guidance. ArduSub owns stabilization and the motor mixer.
class VehicleControl {
 public:
  explicit VehicleControl(rclcpp::Node &node) : node_(node) {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    timeout_ = node_.declare_parameter("command_timeout", 0.5, fixed);
    limit_ = node_.declare_parameter("input_limit", 0.2, fixed);
    source_system_ = node_.declare_parameter("source_system", 255, fixed);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || timeout_ > 1 ||
        !std::isfinite(limit_) || limit_ <= 0 || limit_ > 1 ||
        source_system_ < 1 || source_system_ > 255)
      throw std::invalid_argument("invalid command timeout/input limit/source system");

    auto qos = rclcpp::QoS(1).reliable();
    qos.lifespan(rclcpp::Duration::from_seconds(0.1));
    output_ = node_.create_publisher<mavros_msgs::msg::ManualControl>(
        "/mavros/manual_control/send", qos);
    input_ = node_.create_subscription<aterkia_interfaces::msg::VehicleCommand>(
        "/vehicle/command", rclcpp::QoS(1).reliable(),
        [this](aterkia_interfaces::msg::VehicleCommand::ConstSharedPtr msg) { command(*msg); });
    params_ = node_.create_client<rcl_interfaces::srv::GetParameters>(
        "/mavros/param/get_parameters");
  }

  bool configured() const { return configured_; }
  bool authorized() const { return authorized_; }
  uint64_t generation() const { return generation_; }
  const std::string &configuration_error() const { return configuration_error_; }
  const std::string &reason() const { return reason_; }

  void authorize() {
    authorized_ = true;
    have_command_ = false;
    stamp_ = node_.now().nanoseconds();
    received_ = Clock::now();
    reason_ = "WAITING_COMMAND";
  }

  void stop(const std::string &reason) {
    ++generation_;  // A late mode/arm reply cannot reopen a cancelled session.
    if (authorized_)
      publish(false);
    authorized_ = false;
    have_command_ = false;
    reason_ = reason;
    // Do not keep sending neutral: a dead guidance process must reach FCU failsafe.
  }

  void tick(bool connected, bool sensors_fresh, bool correct_mode, bool armed) {
    check_parameters(connected);
    if (!authorized_)
      return;
    const double age = (node_.now().nanoseconds() - stamp_) / 1e9;
    if (!connected || !sensors_fresh || !correct_mode || !armed || !configured_)
      stop(!connected ? "DISCONNECTED" : !sensors_fresh ? "TELEMETRY_STALE" :
          !correct_mode ? "WRONG_MODE" : !armed ? "DISARMED" : "FCU_CONFIG_UNVERIFIED");
    else if (age < -0.05 || age >= timeout_ ||
        std::chrono::duration<double>(Clock::now() - received_).count() >= timeout_)
      stop("COMMAND_TIMEOUT");
    else
      publish(have_command_);
  }

 private:
  using Clock = std::chrono::steady_clock;
  using GetParameters = rcl_interfaces::srv::GetParameters;

  void command(const aterkia_interfaces::msg::VehicleCommand &msg) {
    if (!authorized_)
      return;
    if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000) {
      stop("INVALID_COMMAND");
      return;
    }
    const std::array<double, 3> values{msg.surge_input, msg.yaw_input, msg.heave_input};
    const int64_t stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    const double age = (node_.now().nanoseconds() - stamp) / 1e9;
    if (msg.header.frame_id != "base_link" || stamp <= stamp_ || age < -0.05 ||
        age >= timeout_ || input_->get_publisher_count() != 1 ||
        std::any_of(values.begin(), values.end(),
            [](double v) { return !std::isfinite(v) || std::abs(v) > 1; })) {
      stop("INVALID_COMMAND");
      return;
    }
    double total = 0;
    for (double v : values)
      total += std::abs(v);
    const double scale = total > limit_ ? limit_ / total : 1.0;
    for (size_t i = 0; i < values.size(); ++i)
      command_[i] = values[i] * scale;
    stamp_ = stamp;
    received_ = Clock::now();
    have_command_ = true;
    reason_ = "ACTIVE";
  }

  void publish(bool moving) {
    mavros_msgs::msg::ManualControl msg;
    msg.header.stamp = node_.now();
    msg.header.frame_id = "base_link";
    // MAVROS forwards these fields unchanged; ArduSub Z is [0,1000], neutral 500.
    msg.x = moving ? std::lround(1000 * command_[0]) : 0;
    msg.y = 0;
    msg.z = moving ? std::lround(500 + 500 * command_[2]) : 500;
    msg.r = moving ? std::lround(-1000 * command_[1]) : 0;
    output_->publish(msg);  // All buttons/extensions remain zero.
  }

  static double number(const rcl_interfaces::msg::ParameterValue &v) {
    if (v.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      return v.integer_value;
    if (v.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
      return v.double_value;
    return std::numeric_limits<double>::quiet_NaN();
  }

  void check_parameters(bool connected) {
    const auto now = Clock::now();
    if (!connected || now - checked_ > std::chrono::seconds(3)) {
      configured_ = false;
      configuration_error_ = "FCU parameter verification unavailable or expired";
    }
    if (query_ && (!connected || now - queried_ >= std::chrono::seconds(2))) {
      params_->remove_pending_request(query_id_);
      query_ = false;
    }
    if (!connected || query_ || now - queried_ < std::chrono::seconds(1) ||
        !params_->service_is_ready())
      return;
    queried_ = now;
    query_ = true;
    auto request = std::make_shared<GetParameters::Request>();
    request->names = {"FS_PILOT_INPUT", "FS_PILOT_TIMEOUT", "MAV_GCS_SYSID", "SYSID_MYGCS",
        "FS_THR_ENABLE", "FRAME_TYPE", "FRAME_CONFIG"};
    const auto query_generation = ++query_generation_;
    const auto future = params_->async_send_request(request,
        [this, query_generation](rclcpp::Client<GetParameters>::SharedFuture reply) {
          if (!query_ || query_generation != query_generation_)
            return;
          query_ = false;
          const auto &v = reply.get()->values;
          checked_ = Clock::now();
          configuration_error_ = v.size() != 7 ? "FCU parameter values unavailable" :
              number(v[0]) != 2 ? "FS_PILOT_INPUT must disarm (2)" :
              !(number(v[1]) > 0 && number(v[1]) <= 2) ? "FS_PILOT_TIMEOUT must be >0 and <=2s" :
              number(v[v[2].type ? 2 : 3]) != source_system_ ? "FCU sender ID does not match source_system" :
              (v[4].type && number(v[4]) != 0) ? "FS_THR_ENABLE conflicts with pilot-input failsafe" :
              number(v[v[5].type ? 5 : 6]) != 2 ? "Vectored-6DoF frame (2) required" : "";
          configured_ = configuration_error_.empty();
        });
    query_id_ = future.request_id;
  }

  rclcpp::Node &node_;
  int source_system_;
  double timeout_, limit_;
  std::array<double, 3> command_{};
  bool authorized_{false}, have_command_{false}, configured_{false}, query_{false};
  std::string reason_{"DISARMED"}, configuration_error_{"Waiting for FCU parameter cache"};
  int64_t stamp_{0}, query_id_{0};
  uint64_t generation_{0}, query_generation_{0};
  Clock::time_point received_{}, queried_{}, checked_{};
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr output_;
  rclcpp::Subscription<aterkia_interfaces::msg::VehicleCommand>::SharedPtr input_;
  rclcpp::Client<GetParameters>::SharedPtr params_;
};
