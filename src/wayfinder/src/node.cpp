#include <algorithm>
#include <array>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <aterkia_interfaces/action/navigate_task.hpp>
#include <aterkia_interfaces/msg/vehicle_command.hpp>
#include <aterkia_interfaces/msg/depth_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

using Task = aterkia_interfaces::action::NavigateTask;
using GoalHandle = rclcpp_action::ServerGoalHandle<Task>;
using Clock = std::chrono::steady_clock;

class Wayfinder : public rclcpp::Node {
 public:
  Wayfinder() : Node("wayfinder") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    const auto vehicle_mode = declare_parameter("vehicle_mode", "mock", fixed);
    hardware_ = vehicle_mode == "hardware";
    if (vehicle_mode != "mock" && !hardware_)
      throw std::invalid_argument("vehicle_mode must be mock or hardware");
    sensor_timeout_ = declare_parameter("sensor_timeout", 0.5, fixed);
    task_timeout_ = declare_parameter("max_task_timeout", 120.0, fixed);
    if (!std::isfinite(sensor_timeout_) || sensor_timeout_ <= 0 || !std::isfinite(task_timeout_) ||
        task_timeout_ <= 0)
      throw std::invalid_argument("timeouts must be finite and positive");

    auto positive = [this, &fixed](const char *name, double value) {
      auto v = declare_parameter(name, value, fixed);
      if (!std::isfinite(v) || v <= 0)
        throw std::invalid_argument(name);
      return v;
    };
    image_width_ = positive("image_width", 640.0);
    image_height_ = positive("image_height", 480.0);
    camera_frame_ = declare_parameter("camera_frame", "camera_optical_frame", fixed);
    calibrated_ = declare_parameter("use_camera_info", false, fixed);
    bearing_tolerance_ = positive("bearing_tolerance", 0.08);
    bearing_gain_ = positive("bearing_gain", 1.0);
    confidence_ = positive("min_confidence", 0.7);
    align_tolerance_ = positive("align_tolerance", 0.08);
    near_width_ = positive("near_width_ratio", 0.55);
    pass_distance_ = positive("pass_distance", 1.2);
    speed_ = positive("surge_speed", 0.4);
    yaw_rate_ = positive("yaw_rate", 0.6);
    turn_tolerance_ = positive("turn_tolerance", 0.05);
    if (confidence_ > 1 || align_tolerance_ >= 1 || near_width_ >= 1 || camera_frame_.empty())
      throw std::invalid_argument("invalid perception/guidance thresholds");

    if (hardware_)
      configure_hardware(fixed);
    else
      command_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1).reliable());
    status_ =
        create_publisher<std_msgs::msg::String>("/navigation/status", rclcpp::QoS(10).reliable());

    if (!hardware_)
      odom_ = create_subscription<nav_msgs::msg::Odometry>("/vehicle/odometry",
        rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000)
            return;
          const auto &q = msg->pose.pose.orientation;
          double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
          double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
          if (msg->header.frame_id != "odom" || msg->child_frame_id != "base_link" ||
              !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01 || age < -0.05 ||
              age > sensor_timeout_ || !std::isfinite(msg->pose.pose.position.x) ||
              !std::isfinite(msg->pose.pose.position.y) ||
              !std::isfinite(msg->pose.pose.position.z) ||
              rclcpp::Time(msg->header.stamp).nanoseconds() <= odom_stamp_)
            return;
          odom_stamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
          yaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
          x_ = msg->pose.pose.position.x;
          y_ = msg->pose.pose.position.y;
          last_odom_ = Clock::now();
          have_odom_ = true;
        });

    // Pair detections with the exact CameraInfo for the same processed frame.
    if (calibrated_) {
      detection_filter_.subscribe(this, "/perception/detections", rclcpp::SensorDataQoS());
      info_filter_.subscribe(this, "/perception/camera_info", rclcpp::SensorDataQoS());
      sync_ = std::make_shared<CameraSync>(5, detection_filter_, info_filter_);
      sync_->registerCallback([this](const vision_msgs::msg::Detection2DArray::ConstSharedPtr &msg,
                                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) {
        on_detections(msg, info);
      });
    } else {
      detections_ = create_subscription<vision_msgs::msg::Detection2DArray>(
          "/perception/detections", rclcpp::SensorDataQoS(),
          [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
            on_detections(msg, nullptr);
          });
    }

    health_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/vehicle/health",
        rclcpp::QoS(1).reliable(),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
          armed_ = false;
          preflight_ = false;
          if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000)
            return;
          double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (age < -0.05 || age > sensor_timeout_ || stamp <= health_stamp_)
            return;
          health_stamp_ = stamp;
          for (const auto &s : msg->status)
            if (s.name == "helm") {
              armed_ = s.level == 0 && s.message == "ARMED";
              for (const auto &field : s.values)
                if (field.key == "preflight_ready")
                  preflight_ = field.value == "true";
            }
          last_health_ = Clock::now();
        });

    readiness_ = create_service<std_srvs::srv::Trigger>("/navigation/ready",
        [this](std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response) {
          response->success = !running_ && !accepting_ && (hardware_ ?
              preflight_ && hardware_sensors_ready() && fresh(last_health_, health_stamp_) &&
                  (!require_camera_ || (calibrated_ && camera_ready())) :
              ready() && camera_ready());
          response->message = response->success ? "Guidance ready" :
              "Guidance requires fresh input for the selected vehicle profile";
        });

    // Reserve accepted goals before execution starts to prevent overlapping tasks.
    action_ = rclcpp_action::create_server<Task>(
        this, "/navigation/task",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Task::Goal> goal) {
          if (running_ || accepting_ || !ready() || !std::isfinite(goal->timeout) ||
              goal->timeout <= 0 || goal->timeout > task_timeout_ ||
              !std::isfinite(goal->turn_radians) ||
              (hardware_ ? goal->task != "DIVE_TO_DEPTH" && goal->task != "TURN" &&
                      goal->task != "APPROACH_GATE" :
                  goal->task != "PASS_GATE" && goal->task != "TURN" && goal->task != "HOLD") ||
              (goal->task == "APPROACH_GATE" && (!calibrated_ || !camera_ready())) ||
              (goal->task == "PASS_GATE" && !camera_ready()) ||
              (goal->task == "TURN" &&
                  (std::abs(goal->turn_radians) < turn_tolerance_ ||
                      std::abs(goal->turn_radians) > std::acos(-1.0))))
            return rclcpp_action::GoalResponse::REJECT;
          accepting_ = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle>) {
          send_command(0, 0);
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> handle) {
          goal_ = handle;
          accepting_ = false;
          running_ = true;
          task_name_ = handle->get_goal()->task;
          active_timeout_ = handle->get_goal()->timeout;
          started_ = Clock::now();
          target_yaw_ = yaw_ + handle->get_goal()->turn_radians;
          stable_samples_ = 0;
          stable_stamp_ = 0;
          phase_ = task_name_ == "PASS_GATE" || task_name_ == "APPROACH_GATE" ? "SEARCH" : task_name_;
          report("RUNNING: " + phase_);
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      tick();
    });
  }

 private:
  void configure_hardware(const rcl_interfaces::msg::ParameterDescriptor &fixed) {
    depth_target_ = declare_parameter("target_depth_m", 1.0, fixed);
    depth_tolerance_ = declare_parameter("depth_tolerance_m", 0.1, fixed);
    depth_gain_ = declare_parameter("depth_gain", 0.5, fixed);
    depth_deviation_ = declare_parameter("depth_abort_error_m", 0.5, fixed);
    heave_limit_ = declare_parameter("heave_input_limit", 0.2, fixed);
    yaw_limit_ = declare_parameter("yaw_input_limit", 0.2, fixed);
    yaw_gain_ = declare_parameter("yaw_input_gain", 0.5, fixed);
    settle_time_ = declare_parameter("settle_time_s", 0.5, fixed);
    require_camera_ = declare_parameter("require_camera_on_start", false, fixed);
    surge_input_ = declare_parameter("surge_input", 0.1, fixed);
    search_input_ = declare_parameter("search_yaw_input", 0.1, fixed);
    bearing_input_gain_ = declare_parameter("bearing_input_gain", 0.5, fixed);
    for (double value : {depth_target_, depth_tolerance_, depth_gain_, depth_deviation_,
             heave_limit_, yaw_limit_, yaw_gain_, settle_time_, surge_input_, search_input_, bearing_input_gain_})
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("hardware guidance parameters must be finite and positive");
    if (heave_limit_ > 1 || yaw_limit_ > 1 || surge_input_ > 1 || search_input_ > yaw_limit_ ||
        (require_camera_ && !calibrated_) || depth_tolerance_ >= depth_target_ ||
        depth_deviation_ <= depth_tolerance_)
      throw std::invalid_argument("invalid hardware guidance limits");
    vehicle_command_ = create_publisher<aterkia_interfaces::msg::VehicleCommand>(
        "/vehicle/command", rclcpp::QoS(1).reliable());
    imu_ = create_subscription<sensor_msgs::msg::Imu>("/vehicle/imu",
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          const auto &q = msg->orientation;
          const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
          if (msg->header.frame_id != "base_link" || msg->orientation_covariance[0] < 0 ||
              !std::isfinite(norm) || std::abs(norm - 1) > 0.01 ||
              !accept_stamp(msg->header.stamp, imu_stamp_, last_imu_))
            return;
          yaw_ = std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
        });
    depth_ = create_subscription<aterkia_interfaces::msg::DepthStamped>(
        "/vehicle/depth_stamped", rclcpp::SensorDataQoS(),
        [this](aterkia_interfaces::msg::DepthStamped::ConstSharedPtr msg) {
          if (msg->header.frame_id == "base_link" && std::isfinite(msg->depth_m) &&
              accept_stamp(msg->header.stamp, depth_stamp_, last_depth_))
            depth_m_ = msg->depth_m;
        });
  }

  bool accept_stamp(const builtin_interfaces::msg::Time &stamp, int64_t &previous,
      Clock::time_point &received) {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000)
      return false;
    const auto ns = rclcpp::Time(stamp).nanoseconds();
    const double age = (now().nanoseconds() - ns) / 1e9;
    if (ns <= previous || age < -0.05 || age >= sensor_timeout_)
      return false;
    previous = ns;
    received = Clock::now();
    return true;
  }

  bool hardware_sensors_ready() const {
    return imu_stamp_ > 0 && depth_stamp_ > 0 && fresh(last_imu_, imu_stamp_) &&
        fresh(last_depth_, depth_stamp_);
  }

  void send_command(double surge, double yaw, double heave = 0) {
    if (hardware_) {
      aterkia_interfaces::msg::VehicleCommand command;
      command.header.stamp = now();
      command.header.frame_id = "base_link";
      command.surge_input = surge;
      command.yaw_input = yaw;
      command.heave_input = heave;
      vehicle_command_->publish(command);
    } else {
      geometry_msgs::msg::Twist command;
      command.linear.x = surge;
      command.angular.z = yaw;
      command_->publish(command);
    }
  }

  bool settled(bool within_tolerance, int64_t source_stamp) {
    if (!within_tolerance) {
      stable_samples_ = 0;
      stable_stamp_ = 0;
      return false;
    }
    if (source_stamp != stable_stamp_) {
      if (stable_samples_++ == 0) {
        stable_since_ = Clock::now();
        stable_first_stamp_ = source_stamp;
      }
      stable_stamp_ = source_stamp;
    }
    return stable_samples_ >= 3 &&
        (source_stamp - stable_first_stamp_) / 1e9 >= settle_time_ &&
        std::chrono::duration<double>(Clock::now() - stable_since_).count() >= settle_time_;
  }

  void tick_hardware() {
    if (!running_ && !armed_)
      return;
    double surge_input = 0, yaw_input = 0, heave_input = 0;
    if (running_) {
      if (goal_->is_canceling()) {
        finish("ABORTED: canceled");
        return;
      }
      if (!ready()) {
        finish("FAILED: telemetry stale or disarmed");
        return;
      }
      if (std::chrono::duration<double>(Clock::now() - started_).count() > active_timeout_) {
        finish("FAILED: timeout");
        return;
      }
      const double depth_error = depth_m_ - depth_target_;
      heave_input = std::abs(depth_error) <= depth_tolerance_ ? 0 :
          std::clamp(depth_gain_ * depth_error, -heave_limit_, heave_limit_);
      if (task_name_ == "DIVE_TO_DEPTH") {
        if (settled(std::abs(depth_error) <= depth_tolerance_, depth_stamp_)) {
          finish("COMPLETED: measured stable depth");
          return;
        }
      } else if (task_name_ == "APPROACH_GATE") {
        if (std::abs(depth_error) > depth_deviation_) {
          finish("FAILED: depth deviation");
          return;
        }
        if (!camera_ready()) {
          finish("FAILED: camera stale");
          return;
        }
        const bool aligned = gate_ && std::abs(error_) <= bearing_tolerance_;
        const bool depth_ok = std::abs(depth_error) <= depth_tolerance_;
        if (settled(aligned && depth_ok && width_ >= near_width_, image_stamp_)) {
          finish("COMPLETED: stable visual approach; gate passage not claimed");
          return;
        }
        if (!gate_) {
          phase("SEARCH");
          yaw_input = search_input_;
        } else {
          yaw_input = std::clamp(-error_ * bearing_input_gain_, -yaw_limit_, yaw_limit_);
          if (!aligned)
            phase("ALIGN");
          else {
            phase("APPROACH");
            // Stop at the near threshold while collecting fresh confirmation images.
            if (depth_ok && width_ < near_width_)
              surge_input = surge_input_ * (1 - std::abs(error_) / bearing_tolerance_);
          }
        }
      } else {
        if (std::abs(depth_error) > depth_deviation_) {
          finish("FAILED: depth deviation");
          return;
        }
        const double error = std::atan2(std::sin(target_yaw_ - yaw_), std::cos(target_yaw_ - yaw_));
        yaw_input = std::abs(error) < turn_tolerance_ ? 0 :
            std::clamp(yaw_gain_ * error, -yaw_limit_, yaw_limit_);
        if (settled(std::abs(error) < turn_tolerance_ &&
                std::abs(depth_error) <= depth_tolerance_, imu_stamp_)) {
          finish("COMPLETED: measured stable heading and depth");
          return;
        }
      }
      auto feedback = std::make_shared<Task::Feedback>();
      feedback->phase = phase_;
      goal_->publish_feedback(feedback);
    }
    send_command(surge_input, yaw_input, heave_input);
  }

  void on_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr &msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) {
    if (info) {
      // Only the full-resolution, monocular rectified geometry emitted by spectra.
      if (info->header != msg->header || info->width != image_width_ ||
          info->height != image_height_ || info->distortion_model != "plumb_bob" ||
          info->d.size() != 5 || info->binning_x > 1 || info->binning_y > 1 || info->roi.x_offset ||
          info->roi.y_offset || info->roi.width || info->roi.height || info->roi.do_rectify)
        return;
      for (double v : info->p)
        if (!std::isfinite(v))
          return;
      for (double v : info->d)
        if (v != 0)
          return;
      const std::array<double, 9> identity{1, 0, 0, 0, 1, 0, 0, 0, 1};
      if (info->r != identity || info->p[0] <= 0 || info->p[5] <= 0 || info->p[2] < 0 ||
          info->p[2] >= image_width_ || info->p[6] < 0 || info->p[6] >= image_height_ ||
          info->p[1] != 0 || info->p[3] != 0 || info->p[4] != 0 || info->p[7] != 0 ||
          info->p[8] != 0 || info->p[9] != 0 || info->p[10] != 1 || info->p[11] != 0)
        return;
      for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
          if (info->k[row * 3 + col] != info->p[row * 4 + col])
            return;
    }
    if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000)
      return;
    double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
    const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (msg->header.frame_id != camera_frame_ || age < -0.05 || age > sensor_timeout_ ||
        stamp <= image_stamp_)
      return;
    image_stamp_ = stamp;
    last_image_ = Clock::now();
    have_image_ = true;
    gate_ = false;

    for (const auto &d : msg->detections) {
      const auto &b = d.bbox;
      if (!std::isfinite(b.center.position.x) || !std::isfinite(b.center.position.y) ||
          !std::isfinite(b.size_x) || !std::isfinite(b.size_y) || b.size_x <= 0 || b.size_y <= 0 ||
          b.size_x > image_width_ || b.size_y > image_height_ || b.center.position.x < 0 ||
          b.center.position.x > image_width_ || b.center.position.y < 0 ||
          b.center.position.y > image_height_)
        continue;
      for (const auto &r : d.results)
        if (r.hypothesis.class_id == "gate" && std::isfinite(r.hypothesis.score) &&
            r.hypothesis.score >= confidence_ && r.hypothesis.score <= 1) {
          gate_ = true;
          error_ = info ? std::atan2(b.center.position.x - info->p[2], info->p[0])
                        : (b.center.position.x - image_width_ / 2) / (image_width_ / 2);
          width_ = b.size_x / image_width_;
        }
    }

    // Low-confidence/malformed detections are not evidence of a clear view.
    empty_images_ = msg->detections.empty() ? std::min(empty_images_ + 1, 3) : 0;
  }

  bool fresh(Clock::time_point received, int64_t stamp) const {
    // Source stamps and monotonic receipt time must both remain fresh.
    double age = (now().nanoseconds() - stamp) / 1e9;
    return age >= -0.05 && age < sensor_timeout_ &&
        std::chrono::duration<double>(Clock::now() - received).count() < sensor_timeout_;
  }

  bool camera_ready() const {
    return have_image_ && fresh(last_image_, image_stamp_);
  }

  void phase(const std::string &next) {
    if (phase_ != next) {
      phase_ = next;
      report("RUNNING: " + phase_);
    }
  }

  bool ready() const {
    return armed_ && (hardware_ ? hardware_sensors_ready() :
        have_odom_ && fresh(last_odom_, odom_stamp_)) &&
        fresh(last_health_, health_stamp_);
  }

  void report(const std::string &text) {
    std_msgs::msg::String msg;
    msg.data = text;
    status_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void finish(const std::string &text) {
    running_ = false;
    send_command(0, 0);
    report(text);
    if (goal_) {
      auto result = std::make_shared<Task::Result>();
      result->status = text.substr(0, text.find(':'));
      result->reason = text;
      if (goal_->is_canceling()) {
        result->status = "ABORTED";
        goal_->canceled(result);
      } else if (result->status == "COMPLETED")
        goal_->succeed(result);
      else
        goal_->abort(result);
      goal_.reset();
    }
  }

  void tick() {
    if (hardware_) {
      tick_hardware();
      return;
    }
    geometry_msgs::msg::Twist cmd;

    // Stop checks precede guidance updates; elapsed time can only fail a task.
    if (running_) {
      if (goal_->is_canceling()) {
        finish("ABORTED: canceled");
        return;
      }
      if (!ready()) {
        finish("FAILED: telemetry stale or disarmed");
        return;
      }
      if (std::chrono::duration<double>(Clock::now() - started_).count() > active_timeout_) {
        finish("FAILED: timeout");
        return;
      }
      if (task_name_ == "TURN") {
        double error = std::atan2(std::sin(target_yaw_ - yaw_), std::cos(target_yaw_ - yaw_));
        if (std::abs(error) < turn_tolerance_) {
          finish("COMPLETED: measured heading");
          return;
        }
        cmd.angular.z = std::clamp(error, -yaw_rate_, yaw_rate_);
      } else if (task_name_ == "PASS_GATE") {
        if (!camera_ready()) {
          finish("FAILED: camera stale");
          return;
        }

        // Gate passage needs measured displacement and fresh clear-view evidence.
        if (phase_ == "PASS") {
          double dx = x_ - pass_x_, dy = y_ - pass_y_;
          double progress = dx * std::cos(pass_yaw_) + dy * std::sin(pass_yaw_);
          double lateral = -dx * std::sin(pass_yaw_) + dy * std::cos(pass_yaw_);
          double heading_error = std::atan2(std::sin(pass_yaw_ - yaw_), std::cos(pass_yaw_ - yaw_));
          if (std::abs(lateral) > 0.3 || std::abs(heading_error) > 0.2) {
            finish("FAILED: pass drift");
            return;
          }
          if (progress >= pass_distance_ && empty_images_ >= 3) {
            finish("COMPLETED: visual acquisition, measured passage and clear view");
            return;
          }
          if (progress > 2 * pass_distance_) {
            finish("FAILED: passage not confirmed");
            return;
          }
          cmd.linear.x = speed_;
          cmd.angular.z = heading_error;
        } else if (!gate_) {
          phase("SEARCH");
          cmd.angular.z = yaw_rate_ / 2;
        } else {
          // Optical image x points right; positive FLU yaw turns left.
          cmd.angular.z =
              std::clamp(-error_ * (calibrated_ ? bearing_gain_ : 1.0), -yaw_rate_, yaw_rate_);
          if (std::abs(error_) > (calibrated_ ? bearing_tolerance_ : align_tolerance_)) {
            phase("ALIGN");
          } else {
            phase("APPROACH");
            cmd.linear.x = speed_;
            if (width_ >= near_width_) {
              pass_x_ = x_;
              pass_y_ = y_;
              pass_yaw_ = yaw_;
              empty_images_ = 0;
              phase("PASS");
            }
          }
        }
      }
    }

    command_->publish(cmd);
    if (goal_) {
      auto feedback = std::make_shared<Task::Feedback>();
      feedback->phase = phase_;
      if (phase_ == "PASS")
        feedback->progress = std::clamp(
            ((x_ - pass_x_) * std::cos(pass_yaw_) + (y_ - pass_y_) * std::sin(pass_yaw_)) /
                pass_distance_,
            0.0, 1.0);
      goal_->publish_feedback(feedback);
    }
  }

  using CameraSync = message_filters::TimeSynchronizer<vision_msgs::msg::Detection2DArray,
      sensor_msgs::msg::CameraInfo>;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detection_filter_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_filter_;
  std::shared_ptr<CameraSync> sync_;

  bool calibrated_{false};
  bool hardware_{false}, preflight_{false}, require_camera_{false};
  double depth_target_, depth_tolerance_, depth_gain_, depth_deviation_;
  double heave_limit_, yaw_limit_, yaw_gain_, settle_time_, depth_m_{0};
  double surge_input_, search_input_, bearing_input_gain_;
  int stable_samples_{0};
  int64_t depth_stamp_{0}, imu_stamp_{0}, stable_stamp_{0}, stable_first_stamp_{0};
  Clock::time_point last_imu_, last_depth_, stable_since_;
  rclcpp::Publisher<aterkia_interfaces::msg::VehicleCommand>::SharedPtr vehicle_command_;
  rclcpp::Subscription<aterkia_interfaces::msg::DepthStamped>::SharedPtr depth_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readiness_;
  double bearing_tolerance_, bearing_gain_;
  double sensor_timeout_, task_timeout_, yaw_{0}, target_yaw_{0};
  bool armed_{false}, have_odom_{false}, running_{false};
  std::string task_name_;
  std::string camera_frame_, phase_;
  double image_width_, image_height_, confidence_, align_tolerance_, near_width_, pass_distance_;
  double speed_, yaw_rate_, turn_tolerance_, x_{0}, y_{0}, error_{0}, width_{0};
  double pass_x_{0}, pass_y_{0}, pass_yaw_{0};
  bool have_image_{false}, gate_{false};
  int empty_images_{0};
  int64_t image_stamp_{0}, odom_stamp_{0}, health_stamp_{0};
  Clock::time_point last_image_;
  Clock::time_point last_odom_, last_health_, started_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_;
  rclcpp_action::Server<Task>::SharedPtr action_;
  std::shared_ptr<GoalHandle> goal_;
  bool accepting_{false};
  double active_timeout_{0};
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Wayfinder>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("wayfinder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
