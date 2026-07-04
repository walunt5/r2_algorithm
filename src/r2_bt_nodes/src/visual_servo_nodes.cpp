#include "r2_bt_nodes/visual_servo_nodes.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <string>

namespace r2_bt_nodes
{

namespace
{

constexpr int kClientSideFailureCode = -1;

const char * feedbackStateToString(std::uint8_t state)
{
  using VisualServo = gmk_visual_servo_interfaces::action::VisualServo;

  switch (state) {
    case VisualServo::Feedback::WAITING_TARGET:
      return "WAITING_TARGET";

    case VisualServo::Feedback::ALIGNING_Y:
      return "ALIGNING_Y";

    case VisualServo::Feedback::ALIGNING_X:
      return "ALIGNING_X";

    case VisualServo::Feedback::SETTLING:
      return "SETTLING";

    default:
      return "UNKNOWN";
  }
}

}  // namespace

R2WeaponVisualServoActionNode::R2WeaponVisualServoActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(12000)),
  request_generation_(0)
{
}

BT::PortsList R2WeaponVisualServoActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/weapon_visual_servo",
      "ROS 2 visual-servo action name"),

    BT::InputPort<double>(
      "target_distance_m",
      0.5,
      "Desired camera-to-target distance in meters"),

    BT::InputPort<double>(
      "timeout_sec",
      10.0,
      "Maximum execution time passed to the action server"),

    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Maximum time to wait for the action server"),

    BT::InputPort<int>(
      "result_grace_ms",
      2000,
      "Additional client-side grace time after timeout_sec"),

    BT::OutputPort<int>(
      "result_code",
      "VisualServo result code; -1 means BT client-side failure"),

    BT::OutputPort<std::string>(
      "result_message",
      "VisualServo result or BT client-side failure message"),

    BT::OutputPort<double>(
      "final_u_error_px",
      "Final horizontal pixel error"),

    BT::OutputPort<double>(
      "final_depth_error_m",
      "Final depth error in meters")
  };
}

void R2WeaponVisualServoActionNode::resetOutputs()
{
  setOutput("result_code", kClientSideFailureCode);
  setOutput("result_message", std::string());
  setOutput("final_u_error_px", 0.0);
  setOutput("final_depth_error_m", 0.0);
}

void R2WeaponVisualServoActionNode::setClientFailureOutputs(
  const std::string & message)
{
  setOutput("result_code", kClientSideFailureCode);
  setOutput("result_message", message);
  setOutput("final_u_error_px", 0.0);
  setOutput("final_depth_error_m", 0.0);
}

void R2WeaponVisualServoActionNode::cancelActiveGoal()
{
  if (!client_ || !goal_handle_) {
    return;
  }

  try {
    client_->async_cancel_goal(goal_handle_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] Failed to request goal cancellation: %s",
      e.what());
  }
}

BT::NodeStatus R2WeaponVisualServoActionNode::onStart()
{
  resetOutputs();

  std::string action_name = "/weapon_visual_servo";
  double target_distance_m = 0.5;
  double timeout_sec = 10.0;
  int server_timeout_ms = 3000;
  int result_grace_ms = 2000;

  getInput("action_name", action_name);
  getInput("target_distance_m", target_distance_m);
  getInput("timeout_sec", timeout_sec);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_grace_ms", result_grace_ms);

  if (action_name.empty()) {
    const std::string message = "action_name must not be empty";
    setClientFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  if (!std::isfinite(target_distance_m) || target_distance_m <= 0.0) {
    const std::string message =
      "target_distance_m must be finite and greater than zero";

    setClientFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s: %.6f",
      message.c_str(),
      target_distance_m);

    return BT::NodeStatus::FAILURE;
  }

  if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
    const std::string message =
      "timeout_sec must be finite and greater than zero";

    setClientFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s: %.6f",
      message.c_str(),
      timeout_sec);

    return BT::NodeStatus::FAILURE;
  }

  if (server_timeout_ms <= 0) {
    const std::string message =
      "server_timeout_ms must be greater than zero";

    setClientFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s: %d",
      message.c_str(),
      server_timeout_ms);

    return BT::NodeStatus::FAILURE;
  }

  if (result_grace_ms < 0) {
    const std::string message =
      "result_grace_ms must be greater than or equal to zero";

    setClientFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s: %d",
      message.c_str(),
      result_grace_ms);

    return BT::NodeStatus::FAILURE;
  }

  const double timeout_ms_double = std::ceil(timeout_sec * 1000.0);

  if (!std::isfinite(timeout_ms_double)) {
    const std::string message =
      "timeout_sec is too large to convert to milliseconds";

    setClientFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  const auto max_duration_ms =
    std::chrono::milliseconds::max().count();

  if (
    timeout_ms_double >
    static_cast<double>(max_duration_ms - result_grace_ms))
  {
    const std::string message =
      "timeout_sec plus result_grace_ms exceeds supported duration";

    setClientFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<VisualServo>(
      node_,
      action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2WeaponVisualServoActionNode] Waiting for action server: %s",
    action_name_.c_str());

  try {
    if (!client_->wait_for_action_server(
        std::chrono::milliseconds(server_timeout_ms)))
    {
      const std::string message =
        "visual-servo action server is not available";

      setClientFailureOutputs(message);
      stage_ = Stage::IDLE;

      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2WeaponVisualServoActionNode] %s: %s",
        message.c_str(),
        action_name_.c_str());

      return BT::NodeStatus::FAILURE;
    }
  } catch (const std::exception & e) {
    const std::string message =
      std::string("exception while waiting for action server: ") + e.what();

    setClientFailureOutputs(message);
    stage_ = Stage::IDLE;

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  const auto timeout_ms =
    static_cast<std::int64_t>(timeout_ms_double);

  result_timeout_ = std::chrono::milliseconds(
    timeout_ms + static_cast<std::int64_t>(result_grace_ms));

  start_time_ = std::chrono::steady_clock::now();
  goal_handle_.reset();

  VisualServo::Goal goal;
  goal.target_distance_m = static_cast<float>(target_distance_m);
  goal.timeout_sec = static_cast<float>(timeout_sec);

  const std::uint64_t generation =
    request_generation_.fetch_add(1) + 1;

  // 捕获本次请求使用的 client。
  // 即使下次执行时 action_name 被修改，也能取消旧服务器上的迟到 Goal。
  const auto request_client = client_;

  auto options =
    rclcpp_action::Client<VisualServo>::SendGoalOptions();

  options.goal_response_callback =
    [this, generation, request_client](GoalHandle::SharedPtr goal_handle) {
      if (!goal_handle) {
        return;
      }

      if (request_generation_.load() != generation) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[R2WeaponVisualServoActionNode] "
          "Late goal acceptance detected; canceling stale goal");

        try {
          request_client->async_cancel_goal(goal_handle);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "[R2WeaponVisualServoActionNode] "
            "Failed to cancel stale goal: %s",
            e.what());
        }
      }
    };

  options.feedback_callback =
    [this, generation](
    GoalHandle::SharedPtr,
    const std::shared_ptr<const VisualServo::Feedback> feedback)
    {
      if (!feedback) {
        return;
      }

      if (request_generation_.load() != generation) {
        return;
      }

      RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "[R2WeaponVisualServoActionNode] "
        "feedback: state=%s(%u) "
        "u=%.2f v=%.2f depth=%.3f confidence=%.3f "
        "error_u=%.2f error_depth=%.3f "
        "cmd_vx=%.3f cmd_vy=%.3f elapsed=%.2f",
        feedbackStateToString(feedback->state),
        static_cast<unsigned int>(feedback->state),
        feedback->u,
        feedback->v,
        feedback->depth_m,
        feedback->confidence,
        feedback->error_u_px,
        feedback->error_depth_m,
        feedback->command_vx_mps,
        feedback->command_vy_mps,
        feedback->elapsed_sec);
    };

  try {
    goal_handle_future_ =
      client_->async_send_goal(goal, options);
  } catch (const std::exception & e) {
    request_generation_.fetch_add(1);

    const std::string message =
      std::string("failed to send visual-servo goal: ") + e.what();

    setClientFailureOutputs(message);
    stage_ = Stage::IDLE;

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  stage_ = Stage::WAIT_GOAL_ACCEPTED;

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2WeaponVisualServoActionNode] Goal sent: "
    "target_distance=%.3f m timeout=%.3f s "
    "client_limit=%ld ms",
    target_distance_m,
    timeout_sec,
    static_cast<long>(result_timeout_.count()));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2WeaponVisualServoActionNode::onRunning()
{
  if (isTimeout()) {
    request_generation_.fetch_add(1);
    cancelActiveGoal();

    const std::string message =
      "visual-servo client result timeout";

    setClientFailureOutputs(message);

    goal_handle_.reset();
    stage_ = Stage::IDLE;

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  if (stage_ == Stage::WAIT_GOAL_ACCEPTED) {
    if (
      goal_handle_future_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }

    try {
      goal_handle_ = goal_handle_future_.get();
    } catch (const std::exception & e) {
      request_generation_.fetch_add(1);

      const std::string message =
        std::string("exception while receiving goal response: ") + e.what();

      setClientFailureOutputs(message);
      stage_ = Stage::IDLE;

      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2WeaponVisualServoActionNode] %s",
        message.c_str());

      return BT::NodeStatus::FAILURE;
    }

    if (!goal_handle_) {
      request_generation_.fetch_add(1);

      const std::string message =
        "visual-servo goal was rejected by the action server";

      setClientFailureOutputs(message);
      stage_ = Stage::IDLE;

      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2WeaponVisualServoActionNode] %s",
        message.c_str());

      return BT::NodeStatus::FAILURE;
    }

    try {
      result_future_ =
        client_->async_get_result(goal_handle_);
    } catch (const std::exception & e) {
      request_generation_.fetch_add(1);
      cancelActiveGoal();

      const std::string message =
        std::string("failed to request visual-servo result: ") + e.what();

      setClientFailureOutputs(message);

      goal_handle_.reset();
      stage_ = Stage::IDLE;

      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2WeaponVisualServoActionNode] %s",
        message.c_str());

      return BT::NodeStatus::FAILURE;
    }

    stage_ = Stage::WAIT_RESULT;

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] "
      "Goal accepted, waiting for result");

    return BT::NodeStatus::RUNNING;
  }

  if (stage_ == Stage::WAIT_RESULT) {
    if (
      result_future_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }

    GoalHandle::WrappedResult wrapped_result;

    try {
      wrapped_result = result_future_.get();
    } catch (const std::exception & e) {
      request_generation_.fetch_add(1);
      cancelActiveGoal();

      const std::string message =
        std::string("exception while receiving visual-servo result: ") +
        e.what();

      setClientFailureOutputs(message);

      goal_handle_.reset();
      stage_ = Stage::IDLE;

      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2WeaponVisualServoActionNode] %s",
        message.c_str());

      return BT::NodeStatus::FAILURE;
    }

    request_generation_.fetch_add(1);
    goal_handle_.reset();
    stage_ = Stage::IDLE;

    const auto & result = wrapped_result.result;

    if (!result) {
      const std::string message =
        "visual-servo result is null";

      setClientFailureOutputs(message);

      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2WeaponVisualServoActionNode] %s",
        message.c_str());

      return BT::NodeStatus::FAILURE;
    }

    setOutput(
      "result_code",
      static_cast<int>(result->code));

    setOutput(
      "result_message",
      result->message);

    setOutput(
      "final_u_error_px",
      static_cast<double>(result->final_u_error_px));

    setOutput(
      "final_depth_error_m",
      static_cast<double>(result->final_depth_error_m));

    const bool ros_action_succeeded =
      wrapped_result.code ==
      rclcpp_action::ResultCode::SUCCEEDED;

    const bool servo_succeeded =
      result->code == VisualServo::Result::SUCCESS;

    if (ros_action_succeeded && servo_succeeded) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2WeaponVisualServoActionNode] SUCCESS: "
        "result_code=%u message=%s "
        "final_u_error=%.2f px final_depth_error=%.3f m",
        static_cast<unsigned int>(result->code),
        result->message.c_str(),
        result->final_u_error_px,
        result->final_depth_error_m);

      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WeaponVisualServoActionNode] FAILURE: "
      "ros_result_code=%d servo_result_code=%u "
      "message=%s final_u_error=%.2f px "
      "final_depth_error=%.3f m",
      static_cast<int>(wrapped_result.code),
      static_cast<unsigned int>(result->code),
      result->message.c_str(),
      result->final_u_error_px,
      result->final_depth_error_m);

    return BT::NodeStatus::FAILURE;
  }

  request_generation_.fetch_add(1);
  cancelActiveGoal();

  const std::string message =
    "invalid internal visual-servo BT node stage";

  setClientFailureOutputs(message);

  goal_handle_.reset();
  stage_ = Stage::IDLE;

  RCLCPP_ERROR(
    node_->get_logger(),
    "[R2WeaponVisualServoActionNode] %s",
    message.c_str());

  return BT::NodeStatus::FAILURE;
}

void R2WeaponVisualServoActionNode::onHalted()
{
  // 先使本次所有异步回调失效。
  request_generation_.fetch_add(1);

  // 如果 Goal 已接受，直接请求取消。
  cancelActiveGoal();

  // 如果 Goal 尚未接受，goal_response_callback 会在其迟到后取消。
  goal_handle_.reset();
  stage_ = Stage::IDLE;

  setClientFailureOutputs(
    "visual-servo BT node halted");

  RCLCPP_WARN(
    node_->get_logger(),
    "[R2WeaponVisualServoActionNode] Halted; cancellation requested");
}

bool R2WeaponVisualServoActionNode::isTimeout() const
{
  return
    (std::chrono::steady_clock::now() - start_time_) >
    result_timeout_;
}

}  // namespace r2_bt_nodes