#include <algorithm>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

using Clock = std::chrono::steady_clock;
using Twist = geometry_msgs::msg::Twist;
class Helm : public rclcpp::Node {
 public:
  Helm() : Node("helm") {
    rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only = true;
    timeout_ = declare_parameter("command_timeout", 0.4, fixed);
    max_speed_ = declare_parameter("max_speed", 0.5, fixed);
    max_yaw_ = declare_parameter("max_yaw_rate", 0.8, fixed);
    declare_parameter("mock_connected", true);
    declare_parameter("mock_telemetry", true);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(max_speed_) || max_speed_ <= 0 ||
        !std::isfinite(max_yaw_) || max_yaw_ <= 0)
      throw std::invalid_argument("vehicle limits and timeout must be finite and positive");
    output_ = create_publisher<Twist>("/vehicle/applied_cmd", rclcpp::QoS(1).reliable());
    odom_ = create_publisher<nav_msgs::msg::Odometry>("/vehicle/odometry", rclcpp::SensorDataQoS());
    depth_ = create_publisher<std_msgs::msg::Float64>("/vehicle/depth", rclcpp::SensorDataQoS());
    heading_ = create_publisher<std_msgs::msg::Float64>("/vehicle/heading", rclcpp::SensorDataQoS());
    health_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/vehicle/health", rclcpp::QoS(1).reliable());
    command_ = create_subscription<Twist>("/cmd_vel", rclcpp::QoS(1).reliable(),
      [this](Twist::ConstSharedPtr msg) {
        if (!armed_) return;
        const double values[] = {msg->linear.x, msg->linear.y, msg->linear.z,
                                 msg->angular.x, msg->angular.y, msg->angular.z};
        for (double v : values) if (!std::isfinite(v)) { stop("invalid command"); return; }
        // Mock supports surge/yaw only; unsupported axes are never silently accepted.
        if (msg->linear.y != 0 || msg->linear.z != 0 || msg->angular.x != 0 || msg->angular.y != 0) {
          stop("unsupported command axis"); return;
        }
        cmd_ = *msg;
        cmd_.linear.x = std::clamp(cmd_.linear.x, -max_speed_, max_speed_);
        cmd_.angular.z = std::clamp(cmd_.angular.z, -max_yaw_, max_yaw_);
        last_cmd_ = Clock::now(); received_ = true;
      });
    arm_ = create_service<std_srvs::srv::SetBool>("/vehicle/arm",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        if (!req->data) { stop("operator disarm"); res->success = true; return; }
        if (!connected() || !get_parameter("mock_telemetry").as_bool()) {
          res->message = "preflight: mock connection/telemetry unavailable"; return;
        }
        if (!armed_) {
          cmd_ = Twist(); received_ = false; armed_ = true;
          RCLCPP_INFO(get_logger(), "ARMED: explicit operator authorization");
        }
        res->success = true;
      });
    abort_ = create_service<std_srvs::srv::Trigger>("/safety/abort",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        stop("operator abort"); res->success = true;
      });
    last_tick_ = Clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "DISARMED: mock only; ENU world / FLU body; depth positive down");
  }
 private:
  bool connected() const { return get_parameter("mock_connected").as_bool(); }
  void stop(const char *reason) {
    if (armed_) RCLCPP_WARN(get_logger(), "DISARMED: %s", reason);
    armed_ = false; received_ = false; cmd_ = Twist(); output_->publish(cmd_);
  }
  void tick() {
    auto now = Clock::now();
    double dt = std::min(0.1, std::chrono::duration<double>(now - last_tick_).count()); last_tick_ = now;
    if (!connected() || !get_parameter("mock_telemetry").as_bool()) stop("telemetry/connection lost");
    if (armed_ && received_ && std::chrono::duration<double>(now - last_cmd_).count() > timeout_)
      stop("command watchdog expired");
    output_->publish(cmd_);
    // Kinematic mock, not an ArduSub adapter or hydrodynamic simulator.
    yaw_ = std::atan2(std::sin(yaw_ + cmd_.angular.z * dt), std::cos(yaw_ + cmd_.angular.z * dt));
    x_ += std::cos(yaw_) * cmd_.linear.x * dt; y_ += std::sin(yaw_) * cmd_.linear.x * dt;
    diagnostic_msgs::msg::DiagnosticArray health; health.header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "helm"; status.hardware_id = "mock";
    status.level = !connected() || !get_parameter("mock_telemetry").as_bool() ? 2 : armed_ ? 0 : 1;
    status.message = status.level == 2 ? "DISCONNECTED" : armed_ ? "ARMED" : "DISARMED";
    health.status.push_back(status); health_->publish(health);
    if (!connected() || !get_parameter("mock_telemetry").as_bool()) return;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now(); odom.header.frame_id = "odom"; odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_; odom.pose.pose.position.y = y_; odom.pose.pose.position.z = -1.0;
    odom.pose.pose.orientation.z = std::sin(yaw_ / 2); odom.pose.pose.orientation.w = std::cos(yaw_ / 2);
    odom.twist.twist = cmd_; odom_->publish(odom);
    std_msgs::msg::Float64 value; value.data = 1.0; depth_->publish(value);
    value.data = yaw_; heading_->publish(value);
  }
  double timeout_, max_speed_, max_yaw_, x_{0}, y_{0}, yaw_{0};
  bool armed_{false}, received_{false};
  Clock::time_point last_cmd_, last_tick_;
  Twist cmd_;
  rclcpp::Publisher<Twist>::SharedPtr output_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_, heading_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_;
  rclcpp::Subscription<Twist>::SharedPtr command_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<Helm>()); rclcpp::shutdown();
}
