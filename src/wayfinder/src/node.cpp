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

using Task = aterkia_interfaces::action::NavigateTask;
using GoalHandle = rclcpp_action::ServerGoalHandle<Task>;
using Clock = std::chrono::steady_clock;

class Wayfinder : public rclcpp::Node {
 public:
  Wayfinder() : Node("wayfinder") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
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

    command_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1).reliable());
    status_ =
        create_publisher<std_msgs::msg::String>("/navigation/status", rclcpp::QoS(10).reliable());

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
          if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000)
            return;
          double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (age < -0.05 || age > sensor_timeout_ || stamp <= health_stamp_)
            return;
          health_stamp_ = stamp;
          for (const auto &s : msg->status)
            if (s.name == "helm")
              armed_ = s.level == 0 && s.message == "ARMED";
          last_health_ = Clock::now();
        });

    // Reserve accepted goals before execution starts to prevent overlapping tasks.
    action_ = rclcpp_action::create_server<Task>(
        this, "/navigation/task",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Task::Goal> goal) {
          if (running_ || accepting_ || !ready() || !std::isfinite(goal->timeout) ||
              goal->timeout <= 0 || goal->timeout > task_timeout_ ||
              !std::isfinite(goal->turn_radians) ||
              (goal->task != "PASS_GATE" && goal->task != "TURN" && goal->task != "HOLD") ||
              (goal->task == "PASS_GATE" && !camera_ready()) ||
              (goal->task == "TURN" &&
                  (std::abs(goal->turn_radians) < turn_tolerance_ ||
                      std::abs(goal->turn_radians) > std::acos(-1.0))))
            return rclcpp_action::GoalResponse::REJECT;
          accepting_ = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle>) {
          command_->publish(geometry_msgs::msg::Twist());
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
          phase_ = task_name_ == "PASS_GATE" ? "SEARCH" : task_name_;
          report("RUNNING: " + phase_);
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      tick();
    });
  }

 private:
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
    return armed_ && have_odom_ && fresh(last_odom_, odom_stamp_) &&
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
    command_->publish(geometry_msgs::msg::Twist());
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
  rclcpp::spin(std::make_shared<Wayfinder>());
  rclcpp::shutdown();
}
