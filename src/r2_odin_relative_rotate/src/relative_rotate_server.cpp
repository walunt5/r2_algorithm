#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "r2_odin_interfaces/action/odin_relative_rotate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace r2_odin_relative_rotate
{

using namespace std::chrono_literals;

class OdinRelativeRotateServer : public rclcpp::Node
{
public:
  using RelativeRotate = r2_odin_interfaces::action::OdinRelativeRotate;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RelativeRotate>;

  OdinRelativeRotateServer()
  : Node("odin_relative_rotate_server")
  {
    action_name_ = declare_parameter<std::string>(
      "action_name", "/r2_odin_relative_rotate");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/odin1/odometry_highfreq");
    cmd_vel_topic_ = declare_parameter<std::string>(
      "cmd_vel_topic", "/cmd_vel");

    control_rate_hz_ = declare_parameter<double>(
      "control_rate_hz", 30.0);
    initial_odom_wait_sec_ = declare_parameter<double>(
      "initial_odom_wait_sec", 2.0);
    odom_timeout_sec_ = declare_parameter<double>(
      "odom_timeout_sec", 0.30);

    yaw_kp_ = declare_parameter<double>("yaw_kp", 1.2);
    min_wz_radps_ = declare_parameter<double>("min_wz_radps", 0.12);
    max_wz_radps_ = declare_parameter<double>("max_wz_radps", 0.45);

    yaw_tolerance_rad_ = declare_parameter<double>(
      "yaw_tolerance_rad", 0.04);
    settle_time_sec_ = declare_parameter<double>(
      "settle_time_sec", 0.25);

    validateParameters();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &OdinRelativeRotateServer::odomCallback,
        this,
        std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10));

    action_server_ = rclcpp_action::create_server<RelativeRotate>(
      this,
      action_name_,
      std::bind(
        &OdinRelativeRotateServer::handleGoal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &OdinRelativeRotateServer::handleCancel,
        this,
        std::placeholders::_1),
      std::bind(
        &OdinRelativeRotateServer::handleAccepted,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Odin relative rotate server started: action=%s odom=%s cmd_vel=%s",
      action_name_.c_str(),
      odom_topic_.c_str(),
      cmd_vel_topic_.c_str());
  }

  ~OdinRelativeRotateServer() override
  {
    shutting_down_.store(true);
    publishStopBurst();

    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  enum class Stage
  {
    ROTATING,
    SETTLING
  };

  struct PoseSample
  {
    double yaw{0.0};
    bool valid{false};
    std::chrono::steady_clock::time_point received_at{};
  };

  struct ActiveGoalGuard
  {
    explicit ActiveGoalGuard(std::atomic_bool & active)
    : active_(active)
    {
    }

    ~ActiveGoalGuard()
    {
      active_.store(false);
    }

    std::atomic_bool & active_;
  };

  void validateParameters()
  {
    if (control_rate_hz_ <= 0.0) {
      throw std::runtime_error("control_rate_hz must be > 0");
    }
    if (initial_odom_wait_sec_ <= 0.0 || odom_timeout_sec_ <= 0.0) {
      throw std::runtime_error("odometry timeout parameters must be > 0");
    }
    if (yaw_tolerance_rad_ <= 0.0 || settle_time_sec_ < 0.0) {
      throw std::runtime_error("invalid yaw tolerance or settle time");
    }
    if (
      yaw_kp_ <= 0.0 ||
      min_wz_radps_ <= 0.0 ||
      max_wz_radps_ < min_wz_radps_)
    {
      throw std::runtime_error("invalid yaw gain or speed parameters");
    }
  }

  static double quaternionToYaw(
    double qx,
    double qy,
    double qz,
    double qw)
  {
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double normalizeAngle(const double angle)
  {
    constexpr double two_pi = 2.0 * 3.14159265358979323846;
    return std::remainder(angle, two_pi);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    PoseSample sample;
    const auto & q = msg->pose.pose.orientation;
    sample.yaw = quaternionToYaw(q.x, q.y, q.z, q.w);
    sample.valid = std::isfinite(sample.yaw);
    sample.received_at = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(pose_mutex_);
    latest_pose_ = sample;
  }

  bool getLatestPose(PoseSample & sample) const
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    sample = latest_pose_;
    return sample.valid;
  }

  bool isPoseFresh(const PoseSample & sample) const
  {
    if (!sample.valid) {
      return false;
    }

    const double age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - sample.received_at).count();

    return age_sec <= odom_timeout_sec_;
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const RelativeRotate::Goal> goal)
  {
    if (active_goal_.load()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: another relative rotate is active");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (
      !std::isfinite(goal->target_yaw_rad) ||
      !std::isfinite(goal->timeout_sec) ||
      goal->timeout_sec <= 0.0)
    {
      RCLCPP_ERROR(get_logger(), "Rejecting goal: invalid numeric fields");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      get_logger(),
      "Accepting relative rotate: target_yaw=%.3f rad timeout=%.3f s",
      goal->target_yaw_rad,
      goal->timeout_sec);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "Cancel request accepted");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }

    active_goal_.store(true);
    worker_ = std::thread(
      &OdinRelativeRotateServer::execute,
      this,
      goal_handle);
  }

  double calculateAngularSpeed(const double yaw_error) const
  {
    if (std::abs(yaw_error) <= yaw_tolerance_rad_) {
      return 0.0;
    }

    double speed = yaw_kp_ * std::abs(yaw_error);
    speed = std::clamp(speed, min_wz_radps_, max_wz_radps_);
    return std::copysign(speed, yaw_error);
  }

  geometry_msgs::msg::Twist makeYawCommand(const double wz) const
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = wz;
    return cmd;
  }

  void publishCommand(const geometry_msgs::msg::Twist & cmd)
  {
    cmd_vel_pub_->publish(cmd);
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }

  void publishStopBurst()
  {
    if (!cmd_vel_pub_) {
      return;
    }

    for (int i = 0; i < 3; ++i) {
      publishStop();
      std::this_thread::sleep_for(20ms);
    }
  }

  void finishSucceeded(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const double yaw_error)
  {
    publishStopBurst();

    auto result = std::make_shared<RelativeRotate::Result>();
    result->code = RelativeRotate::Result::SUCCESS;
    result->message = "relative rotate completed";
    result->final_yaw_error_rad = yaw_error;

    goal_handle->succeed(result);

    RCLCPP_INFO(
      get_logger(),
      "Relative rotate succeeded: final_yaw_error=%.3f rad",
      yaw_error);
  }

  void finishAborted(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::uint8_t code,
    const std::string & message,
    const double yaw_error)
  {
    publishStopBurst();

    auto result = std::make_shared<RelativeRotate::Result>();
    result->code = code;
    result->message = message;
    result->final_yaw_error_rad = yaw_error;

    goal_handle->abort(result);

    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

  void finishCanceled(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const double yaw_error)
  {
    publishStopBurst();

    auto result = std::make_shared<RelativeRotate::Result>();
    result->code = RelativeRotate::Result::CANCELED;
    result->message = "relative rotate canceled";
    result->final_yaw_error_rad = yaw_error;

    goal_handle->canceled(result);
    RCLCPP_WARN(get_logger(), "Relative rotate canceled");
  }

  void publishFeedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::uint8_t stage,
    const double rotated_yaw,
    const double yaw_error,
    const double command_wz,
    const double elapsed_sec)
  {
    auto feedback = std::make_shared<RelativeRotate::Feedback>();
    feedback->stage = stage;
    feedback->rotated_yaw_rad = rotated_yaw;
    feedback->yaw_error_rad = yaw_error;
    feedback->command_wz_radps = command_wz;
    feedback->elapsed_sec = elapsed_sec;
    goal_handle->publish_feedback(feedback);
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    ActiveGoalGuard active_guard(active_goal_);

    const auto goal = goal_handle->get_goal();
    const auto start_time = std::chrono::steady_clock::now();

    rclcpp::WallRate rate(control_rate_hz_);

    PoseSample previous_pose;
    bool start_pose_ready = false;

    while (rclcpp::ok() && !shutting_down_.load()) {
      if (goal_handle->is_canceling()) {
        finishCanceled(goal_handle, goal->target_yaw_rad);
        return;
      }

      const double wait_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();

      if (wait_sec > initial_odom_wait_sec_) {
        finishAborted(
          goal_handle,
          RelativeRotate::Result::ODOM_TIMEOUT,
          "no fresh odometry received before start",
          goal->target_yaw_rad);
        return;
      }

      if (getLatestPose(previous_pose) && isPoseFresh(previous_pose)) {
        start_pose_ready = true;
        break;
      }

      publishFeedback(
        goal_handle,
        RelativeRotate::Feedback::WAITING_ODOM,
        0.0,
        goal->target_yaw_rad,
        0.0,
        wait_sec);

      rate.sleep();
    }

    if (!start_pose_ready) {
      finishAborted(
        goal_handle,
        RelativeRotate::Result::INTERNAL_ERROR,
        "server stopped while waiting for odometry",
        goal->target_yaw_rad);
      return;
    }

    double rotated_yaw = 0.0;
    double yaw_error = goal->target_yaw_rad;

    if (std::abs(yaw_error) <= yaw_tolerance_rad_) {
      finishSucceeded(goal_handle, yaw_error);
      return;
    }

    Stage stage = Stage::ROTATING;
    auto settle_started = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !shutting_down_.load()) {
      if (goal_handle->is_canceling()) {
        finishCanceled(goal_handle, yaw_error);
        return;
      }

      const auto now_steady = std::chrono::steady_clock::now();
      const double elapsed_sec =
        std::chrono::duration<double>(now_steady - start_time).count();

      if (elapsed_sec > goal->timeout_sec) {
        finishAborted(
          goal_handle,
          RelativeRotate::Result::ACTION_TIMEOUT,
          "relative rotate action timeout",
          yaw_error);
        return;
      }

      PoseSample current_pose;
      if (!getLatestPose(current_pose) || !isPoseFresh(current_pose)) {
        finishAborted(
          goal_handle,
          RelativeRotate::Result::ODOM_TIMEOUT,
          "odometry became unavailable or stale",
          yaw_error);
        return;
      }

      rotated_yaw += normalizeAngle(current_pose.yaw - previous_pose.yaw);
      previous_pose = current_pose;
      yaw_error = goal->target_yaw_rad - rotated_yaw;

      switch (stage) {
        case Stage::ROTATING:
        {
          if (std::abs(yaw_error) <= yaw_tolerance_rad_) {
            publishStop();
            publishFeedback(
              goal_handle,
              RelativeRotate::Feedback::SETTLING,
              rotated_yaw,
              yaw_error,
              0.0,
              elapsed_sec);
            settle_started = now_steady;
            stage = Stage::SETTLING;
            break;
          }

          const double wz = calculateAngularSpeed(yaw_error);
          const auto cmd = makeYawCommand(wz);
          publishCommand(cmd);
          publishFeedback(
            goal_handle,
            RelativeRotate::Feedback::ROTATING,
            rotated_yaw,
            yaw_error,
            wz,
            elapsed_sec);
          break;
        }

        case Stage::SETTLING:
        {
          publishStop();
          publishFeedback(
            goal_handle,
            RelativeRotate::Feedback::SETTLING,
            rotated_yaw,
            yaw_error,
            0.0,
            elapsed_sec);

          const double settle_elapsed = std::chrono::duration<double>(
            now_steady - settle_started).count();

          if (std::abs(yaw_error) > yaw_tolerance_rad_) {
            stage = Stage::ROTATING;
          } else if (settle_elapsed >= settle_time_sec_) {
            finishSucceeded(goal_handle, yaw_error);
            return;
          }
          break;
        }
      }

      rate.sleep();
    }

    finishAborted(
      goal_handle,
      RelativeRotate::Result::INTERNAL_ERROR,
      "relative rotate server stopped",
      yaw_error);
  }

  std::string action_name_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;

  double control_rate_hz_{30.0};
  double initial_odom_wait_sec_{2.0};
  double odom_timeout_sec_{0.30};

  double yaw_kp_{1.2};
  double min_wz_radps_{0.12};
  double max_wz_radps_{0.45};

  double yaw_tolerance_rad_{0.04};
  double settle_time_sec_{0.25};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Server<RelativeRotate>::SharedPtr action_server_;

  mutable std::mutex pose_mutex_;
  PoseSample latest_pose_;

  std::atomic_bool active_goal_{false};
  std::atomic_bool shutting_down_{false};
  std::thread worker_;
};

}  // namespace r2_odin_relative_rotate

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<r2_odin_relative_rotate::OdinRelativeRotateServer>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("odin_relative_rotate_server"),
      "Fatal error: %s",
      e.what());
  }

  rclcpp::shutdown();
  return 0;
}
