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
#include "r2_odin_interfaces/action/odin_relative_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace r2_odin_relative_move
{

using namespace std::chrono_literals;

class OdinRelativeMoveServer : public rclcpp::Node
{
public:
  using RelativeMove = r2_odin_interfaces::action::OdinRelativeMove;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RelativeMove>;

  OdinRelativeMoveServer()
  : Node("odin_relative_move_server")
  {
    action_name_ = declare_parameter<std::string>(
      "action_name", "/r2_odin_relative_move");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 30.0);
    initial_odom_wait_sec_ = declare_parameter<double>("initial_odom_wait_sec", 2.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.30);

    forward_kp_ = declare_parameter<double>("forward_kp", 0.8);
    lateral_kp_ = declare_parameter<double>("lateral_kp", 0.8);

    min_forward_speed_mps_ =
      declare_parameter<double>("min_forward_speed_mps", 0.08);
    max_forward_speed_mps_ =
      declare_parameter<double>("max_forward_speed_mps", 0.30);

    min_lateral_speed_mps_ =
      declare_parameter<double>("min_lateral_speed_mps", 0.08);
    max_lateral_speed_mps_ =
      declare_parameter<double>("max_lateral_speed_mps", 0.25);

    position_tolerance_m_ =
      declare_parameter<double>("position_tolerance_m", 0.05);
    settle_time_sec_ = declare_parameter<double>("settle_time_sec", 0.25);

    validateParameters();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &OdinRelativeMoveServer::odomCallback,
        this,
        std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10));

    action_server_ = rclcpp_action::create_server<RelativeMove>(
      this,
      action_name_,
      std::bind(
        &OdinRelativeMoveServer::handleGoal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &OdinRelativeMoveServer::handleCancel,
        this,
        std::placeholders::_1),
      std::bind(
        &OdinRelativeMoveServer::handleAccepted,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Odin relative move server started: action=%s odom=%s cmd_vel=%s",
      action_name_.c_str(),
      odom_topic_.c_str(),
      cmd_vel_topic_.c_str());
  }

  ~OdinRelativeMoveServer() override
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
    MOVING_FORWARD,
    SETTLING_FORWARD,
    MOVING_LATERAL,
    SETTLING_LATERAL
  };

  struct PoseSample
  {
    double x{0.0};
    double y{0.0};
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
    if (position_tolerance_m_ <= 0.0 || settle_time_sec_ < 0.0) {
      throw std::runtime_error("invalid tolerance or settle time");
    }
    if (
      min_forward_speed_mps_ <= 0.0 ||
      max_forward_speed_mps_ < min_forward_speed_mps_ ||
      min_lateral_speed_mps_ <= 0.0 ||
      max_lateral_speed_mps_ < min_lateral_speed_mps_)
    {
      throw std::runtime_error("invalid minimum/maximum speed parameters");
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

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    PoseSample sample;
    sample.x = msg->pose.pose.position.x;
    sample.y = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    sample.yaw = quaternionToYaw(q.x, q.y, q.z, q.w);

    sample.valid =
      std::isfinite(sample.x) &&
      std::isfinite(sample.y) &&
      std::isfinite(sample.yaw);

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
    std::shared_ptr<const RelativeMove::Goal> goal)
  {
    if (active_goal_.load()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: another relative move is active");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (
      !std::isfinite(goal->forward_m) ||
      !std::isfinite(goal->lateral_m) ||
      !std::isfinite(goal->timeout_sec) ||
      goal->timeout_sec <= 0.0)
    {
      RCLCPP_ERROR(get_logger(), "Rejecting goal: invalid numeric fields");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      get_logger(),
      "Accepting relative move: forward=%.3f m lateral=%.3f m timeout=%.3f s",
      goal->forward_m,
      goal->lateral_m,
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
      &OdinRelativeMoveServer::execute,
      this,
      goal_handle);
  }

  double calculateSpeed(
    double error,
    double kp,
    double min_speed,
    double max_speed) const
  {
    if (std::abs(error) <= position_tolerance_m_) {
      return 0.0;
    }

    double speed = kp * std::abs(error);
    speed = std::clamp(speed, min_speed, max_speed);
    return std::copysign(speed, error);
  }

  geometry_msgs::msg::Twist makeBodyCommand(
    double vx,
    double vy) const
  {
    geometry_msgs::msg::Twist cmd;

    // Strict polyline mode:
    // forward stage only uses linear.x;
    // lateral stage only uses linear.y;
    // no yaw command is ever generated.
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.linear.z = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

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

  static void setFeedbackCommand(
    RelativeMove::Feedback & feedback,
    const geometry_msgs::msg::Twist & cmd)
  {
    feedback.command_vx_mps = cmd.linear.x;
    feedback.command_vy_mps = cmd.linear.y;
  }

  void finishSucceeded(
    const std::shared_ptr<GoalHandle> & goal_handle,
    double forward_error,
    double lateral_error)
  {
    publishStopBurst();

    auto result = std::make_shared<RelativeMove::Result>();
    result->code = RelativeMove::Result::SUCCESS;
    result->message = "relative move completed";
    result->final_forward_error_m = forward_error;
    result->final_lateral_error_m = lateral_error;

    goal_handle->succeed(result);

    RCLCPP_INFO(
      get_logger(),
      "Relative move succeeded: final_error=(%.3f, %.3f)",
      forward_error,
      lateral_error);
  }

  void finishAborted(
    const std::shared_ptr<GoalHandle> & goal_handle,
    std::uint8_t code,
    const std::string & message,
    double forward_error,
    double lateral_error)
  {
    publishStopBurst();

    auto result = std::make_shared<RelativeMove::Result>();
    result->code = code;
    result->message = message;
    result->final_forward_error_m = forward_error;
    result->final_lateral_error_m = lateral_error;

    goal_handle->abort(result);

    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

  void finishCanceled(
    const std::shared_ptr<GoalHandle> & goal_handle,
    double forward_error,
    double lateral_error)
  {
    publishStopBurst();

    auto result = std::make_shared<RelativeMove::Result>();
    result->code = RelativeMove::Result::CANCELED;
    result->message = "relative move canceled";
    result->final_forward_error_m = forward_error;
    result->final_lateral_error_m = lateral_error;

    goal_handle->canceled(result);
    RCLCPP_WARN(get_logger(), "Relative move canceled");
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    ActiveGoalGuard active_guard(active_goal_);

    const auto goal = goal_handle->get_goal();
    const auto start_time = std::chrono::steady_clock::now();

    rclcpp::WallRate rate(control_rate_hz_);

    PoseSample start_pose;
    bool start_pose_ready = false;

    while (rclcpp::ok() && !shutting_down_.load()) {
      if (goal_handle->is_canceling()) {
        finishCanceled(
          goal_handle,
          goal->forward_m,
          goal->lateral_m);
        return;
      }

      const double wait_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();

      if (wait_sec > initial_odom_wait_sec_) {
        finishAborted(
          goal_handle,
          RelativeMove::Result::ODOM_TIMEOUT,
          "no fresh odometry received before start",
          goal->forward_m,
          goal->lateral_m);
        return;
      }

      if (getLatestPose(start_pose) && isPoseFresh(start_pose)) {
        start_pose_ready = true;
        break;
      }

      auto feedback = std::make_shared<RelativeMove::Feedback>();
      feedback->stage = RelativeMove::Feedback::WAITING_ODOM;
      feedback->forward_error_m = goal->forward_m;
      feedback->lateral_error_m = goal->lateral_m;
      feedback->elapsed_sec = wait_sec;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    if (!start_pose_ready) {
      finishAborted(
        goal_handle,
        RelativeMove::Result::INTERNAL_ERROR,
        "server stopped while waiting for odometry",
        goal->forward_m,
        goal->lateral_m);
      return;
    }

    if (
      std::abs(goal->forward_m) <= position_tolerance_m_ &&
      std::abs(goal->lateral_m) <= position_tolerance_m_)
    {
      finishSucceeded(
        goal_handle,
        goal->forward_m,
        goal->lateral_m);
      return;
    }

    Stage stage =
      std::abs(goal->forward_m) > position_tolerance_m_ ?
      Stage::MOVING_FORWARD :
      Stage::MOVING_LATERAL;

    auto settle_started = std::chrono::steady_clock::now();

    double forward_error = goal->forward_m;
    double lateral_error = goal->lateral_m;

    while (rclcpp::ok() && !shutting_down_.load()) {
      if (goal_handle->is_canceling()) {
        finishCanceled(goal_handle, forward_error, lateral_error);
        return;
      }

      const auto now_steady = std::chrono::steady_clock::now();
      const double elapsed_sec =
        std::chrono::duration<double>(now_steady - start_time).count();

      if (elapsed_sec > goal->timeout_sec) {
        finishAborted(
          goal_handle,
          RelativeMove::Result::ACTION_TIMEOUT,
          "relative move action timeout",
          forward_error,
          lateral_error);
        return;
      }

      PoseSample current_pose;
      if (!getLatestPose(current_pose) || !isPoseFresh(current_pose)) {
        finishAborted(
          goal_handle,
          RelativeMove::Result::ODOM_TIMEOUT,
          "odometry became unavailable or stale",
          forward_error,
          lateral_error);
        return;
      }

      const double dx = current_pose.x - start_pose.x;
      const double dy = current_pose.y - start_pose.y;

      const double moved_forward =
        std::cos(start_pose.yaw) * dx +
        std::sin(start_pose.yaw) * dy;

      const double moved_lateral =
        -std::sin(start_pose.yaw) * dx +
        std::cos(start_pose.yaw) * dy;

      forward_error = goal->forward_m - moved_forward;
      lateral_error = goal->lateral_m - moved_lateral;

      auto feedback = std::make_shared<RelativeMove::Feedback>();
      feedback->moved_forward_m = moved_forward;
      feedback->moved_lateral_m = moved_lateral;
      feedback->forward_error_m = forward_error;
      feedback->lateral_error_m = lateral_error;
      feedback->elapsed_sec = elapsed_sec;

      geometry_msgs::msg::Twist cmd;

      switch (stage) {
        case Stage::MOVING_FORWARD:
        {
          feedback->stage = RelativeMove::Feedback::MOVING_FORWARD;

          if (std::abs(forward_error) <= position_tolerance_m_) {
            publishStop();
            settle_started = now_steady;
            stage = Stage::SETTLING_FORWARD;
            break;
          }

          const double vx_start = calculateSpeed(
            forward_error,
            forward_kp_,
            min_forward_speed_mps_,
            max_forward_speed_mps_);

          // Strict forward segment: only VX is allowed.
          const double vy_start = 0.0;

          cmd = makeBodyCommand(
            vx_start,
            vy_start);

          publishCommand(cmd);
          setFeedbackCommand(*feedback, cmd);
          break;
        }

        case Stage::SETTLING_FORWARD:
        {
          feedback->stage = RelativeMove::Feedback::SETTLING_FORWARD;
          publishStop();

          const double settle_elapsed = std::chrono::duration<double>(
            now_steady - settle_started).count();

          if (settle_elapsed >= settle_time_sec_) {
            if (std::abs(forward_error) > position_tolerance_m_) {
              stage = Stage::MOVING_FORWARD;
            } else if (
              std::abs(goal->lateral_m) > position_tolerance_m_)
            {
              stage = Stage::MOVING_LATERAL;
            } else {
              finishSucceeded(
                goal_handle,
                forward_error,
                lateral_error);
              return;
            }
          }
          break;
        }

        case Stage::MOVING_LATERAL:
        {
          feedback->stage = RelativeMove::Feedback::MOVING_LATERAL;

          if (std::abs(lateral_error) <= position_tolerance_m_) {
            publishStop();
            settle_started = now_steady;
            stage = Stage::SETTLING_LATERAL;
            break;
          }

          const double vy_start = calculateSpeed(
            lateral_error,
            lateral_kp_,
            min_lateral_speed_mps_,
            max_lateral_speed_mps_);

          // Strict lateral segment: only VY is allowed.
          const double vx_start = 0.0;

          cmd = makeBodyCommand(
            vx_start,
            vy_start);

          publishCommand(cmd);
          setFeedbackCommand(*feedback, cmd);
          break;
        }

        case Stage::SETTLING_LATERAL:
        {
          feedback->stage = RelativeMove::Feedback::SETTLING_LATERAL;
          publishStop();

          const double settle_elapsed = std::chrono::duration<double>(
            now_steady - settle_started).count();

          if (settle_elapsed >= settle_time_sec_) {
            if (std::abs(lateral_error) > position_tolerance_m_) {
              stage = Stage::MOVING_LATERAL;
            } else {
              finishSucceeded(
                goal_handle,
                forward_error,
                lateral_error);
              return;
            }
          }
          break;
        }
      }

      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    finishAborted(
      goal_handle,
      RelativeMove::Result::INTERNAL_ERROR,
      "relative move server stopped",
      forward_error,
      lateral_error);
  }

  std::string action_name_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;

  double control_rate_hz_{30.0};
  double initial_odom_wait_sec_{2.0};
  double odom_timeout_sec_{0.30};

  double forward_kp_{0.8};
  double lateral_kp_{0.8};

  double min_forward_speed_mps_{0.08};
  double max_forward_speed_mps_{0.30};

  double min_lateral_speed_mps_{0.08};
  double max_lateral_speed_mps_{0.25};

  double position_tolerance_m_{0.05};
  double settle_time_sec_{0.25};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Server<RelativeMove>::SharedPtr action_server_;

  mutable std::mutex pose_mutex_;
  PoseSample latest_pose_;

  std::atomic_bool active_goal_{false};
  std::atomic_bool shutting_down_{false};
  std::thread worker_;
};

}  // namespace r2_odin_relative_move

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<r2_odin_relative_move::OdinRelativeMoveServer>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("odin_relative_move_server"),
      "Fatal error: %s",
      e.what());
  }

  rclcpp::shutdown();
  return 0;
}