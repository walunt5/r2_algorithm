#include "r2_bt_nodes/odin_relative_rotate_nodes.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <string>

namespace r2_bt_nodes
{

namespace
{

constexpr int kClientFailureCode = -1;

}  // namespace

R2OdinRelativeRotateActionNode::
R2OdinRelativeRotateActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
}

BT::PortsList
R2OdinRelativeRotateActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_odin_relative_rotate",
      "ROS 2 Odin relative rotate action name"),

    BT::InputPort<double>(
      "target_yaw_rad",
      0.0,
      "Relative yaw rotation in radians; positive left, negative right"),

    BT::InputPort<double>(
      "timeout_sec",
      8.0,
      "Server-side rotation timeout"),

    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Maximum time waiting for action server"),

    BT::InputPort<int>(
      "result_grace_ms",
      2000,
      "Additional BT client result timeout"),

    BT::OutputPort<int>(
      "result_code",
      "Action result code; -1 means BT client failure"),

    BT::OutputPort<std::string>(
      "result_message",
      "Action result message"),

    BT::OutputPort<double>(
      "final_yaw_error_rad",
      "Final relative yaw error")
  };
}

void R2OdinRelativeRotateActionNode::resetOutputs()
{
  setOutput(
    "result_code",
    kClientFailureCode);

  setOutput(
    "result_message",
    std::string());

  setOutput(
    "final_yaw_error_rad",
    0.0);
}

void R2OdinRelativeRotateActionNode::setFailureOutputs(
  const std::string & message)
{
  setOutput(
    "result_code",
    kClientFailureCode);

  setOutput(
    "result_message",
    message);

  setOutput(
    "final_yaw_error_rad",
    0.0);
}

void R2OdinRelativeRotateActionNode::cancelActiveGoal()
{
  if (!client_ || !goal_handle_) {
    return;
  }

  try {
    client_->async_cancel_goal(goal_handle_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] "
      "Failed to cancel goal: %s",
      e.what());
  }
}

BT::NodeStatus
R2OdinRelativeRotateActionNode::onStart()
{
  resetOutputs();

  stage_ = Stage::IDLE;
  goal_handle_.reset();

  std::string action_name =
    "/r2_odin_relative_rotate";

  double target_yaw_rad = 0.0;
  double timeout_sec = 8.0;

  int server_timeout_ms = 3000;
  int result_grace_ms = 2000;

  getInput("action_name", action_name);
  getInput("target_yaw_rad", target_yaw_rad);
  getInput("timeout_sec", timeout_sec);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_grace_ms", result_grace_ms);

  if (action_name.empty()) {
    const std::string message =
      "action_name must not be empty";

    setFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  if (!std::isfinite(target_yaw_rad)) {
    const std::string message =
      "target_yaw_rad must be finite";

    setFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  if (
    !std::isfinite(timeout_sec) ||
    timeout_sec <= 0.0)
  {
    const std::string message =
      "timeout_sec must be finite and greater than zero";

    setFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  if (server_timeout_ms <= 0) {
    const std::string message =
      "server_timeout_ms must be greater than zero";

    setFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  if (result_grace_ms < 0) {
    const std::string message =
      "result_grace_ms must not be negative";

    setFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  if (std::abs(target_yaw_rad) < 1e-6) {
    setOutput(
      "result_code",
      static_cast<int>(
        OdinRelativeRotate::Result::SUCCESS));

    setOutput(
      "result_message",
      std::string("zero-rotation goal"));

    return BT::NodeStatus::SUCCESS;
  }

  if (
    !client_ ||
    action_name_ != action_name)
  {
    action_name_ = action_name;

    client_ =
      rclcpp_action::create_client<
        OdinRelativeRotate>(
          node_,
          action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2OdinRelativeRotateActionNode] "
    "Waiting for action server: %s",
    action_name_.c_str());

  if (
    !client_->wait_for_action_server(
      std::chrono::milliseconds(
        server_timeout_ms)))
  {
    const std::string message =
      "relative rotate action server unavailable";

    setFailureOutputs(message);

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] "
      "%s: %s",
      message.c_str(),
      action_name_.c_str());

    return BT::NodeStatus::FAILURE;
  }

  const double result_timeout_ms_double =
    timeout_sec * 1000.0 +
    static_cast<double>(result_grace_ms);

  result_timeout_ =
    std::chrono::milliseconds(
      static_cast<std::int64_t>(
        std::ceil(
          result_timeout_ms_double)));

  start_time_ =
    std::chrono::steady_clock::now();

  auto goal =
    OdinRelativeRotate::Goal();

  goal.target_yaw_rad = target_yaw_rad;
  goal.timeout_sec = timeout_sec;

  auto options =
    rclcpp_action::Client<
      OdinRelativeRotate>::SendGoalOptions();

  options.feedback_callback =
    [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<
        const OdinRelativeRotate::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "[R2OdinRelativeRotateActionNode] "
        "stage=%u rotated=%.3f error=%.3f cmd_wz=%.3f",
        static_cast<unsigned int>(
          feedback->stage),
        feedback->rotated_yaw_rad,
        feedback->yaw_error_rad,
        feedback->command_wz_radps);
    };

  goal_handle_future_ =
    client_->async_send_goal(
      goal,
      options);

  stage_ =
    Stage::WAIT_GOAL_ACCEPTED;

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2OdinRelativeRotateActionNode] "
    "Goal sent: target_yaw=%.3f rad, timeout=%.2f s",
    target_yaw_rad,
    timeout_sec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
R2OdinRelativeRotateActionNode::onRunning()
{
  if (isTimeout()) {
    cancelActiveGoal();

    const std::string message =
      "BT client timeout while waiting for result";

    setFailureOutputs(message);

    stage_ = Stage::IDLE;
    goal_handle_.reset();

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] %s",
      message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  if (
    stage_ ==
    Stage::WAIT_GOAL_ACCEPTED)
  {
    const auto status =
      goal_handle_future_.wait_for(
        std::chrono::milliseconds(0));

    if (
      status !=
      std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }

    goal_handle_ =
      goal_handle_future_.get();

    if (!goal_handle_) {
      const std::string message =
        "relative rotate goal rejected";

      setFailureOutputs(message);

      stage_ = Stage::IDLE;

      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2OdinRelativeRotateActionNode] %s",
        message.c_str());

      return BT::NodeStatus::FAILURE;
    }

    result_future_ =
      client_->async_get_result(
        goal_handle_);

    stage_ =
      Stage::WAIT_RESULT;

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] "
      "Goal accepted");

    return BT::NodeStatus::RUNNING;
  }

  if (
    stage_ ==
    Stage::WAIT_RESULT)
  {
    const auto status =
      result_future_.wait_for(
        std::chrono::milliseconds(0));

    if (
      status !=
      std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }

    const auto wrapped_result =
      result_future_.get();

    stage_ = Stage::IDLE;
    goal_handle_.reset();

    if (!wrapped_result.result) {
      const std::string message =
        "relative rotate returned null result";

      setFailureOutputs(message);

      return BT::NodeStatus::FAILURE;
    }

    setOutput(
      "result_code",
      static_cast<int>(
        wrapped_result.result->code));

    setOutput(
      "result_message",
      wrapped_result.result->message);

    setOutput(
      "final_yaw_error_rad",
      wrapped_result.result->
        final_yaw_error_rad);

    const bool action_succeeded =
      wrapped_result.code ==
      rclcpp_action::ResultCode::SUCCEEDED;

    const bool controller_succeeded =
      wrapped_result.result->code ==
      OdinRelativeRotate::Result::SUCCESS;

    if (
      action_succeeded &&
      controller_succeeded)
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2OdinRelativeRotateActionNode] "
        "Succeeded: %s, final_yaw_error=%.3f",
        wrapped_result.result->
          message.c_str(),
        wrapped_result.result->
          final_yaw_error_rad);

      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeRotateActionNode] "
      "Failed: action_code=%d, controller_code=%u, message=%s",
      static_cast<int>(
        wrapped_result.code),
      static_cast<unsigned int>(
        wrapped_result.result->code),
      wrapped_result.result->
        message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}

void R2OdinRelativeRotateActionNode::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[R2OdinRelativeRotateActionNode] "
    "Halted, canceling active goal");

  cancelActiveGoal();

  goal_handle_.reset();
  stage_ = Stage::IDLE;
}

bool R2OdinRelativeRotateActionNode::isTimeout() const
{
  if (stage_ == Stage::IDLE) {
    return false;
  }

  return (
    std::chrono::steady_clock::now() -
    start_time_) > result_timeout_;
}

}  // namespace r2_bt_nodes
