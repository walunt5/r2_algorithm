#include "r2_bt_nodes/odin_relative_move_nodes.hpp"

#include <cmath>
#include <exception>
#include <string>

namespace r2_bt_nodes
{

namespace
{
constexpr int kClientFailureCode = -1;
}

R2OdinRelativeMoveActionNode::R2OdinRelativeMoveActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
}

BT::PortsList R2OdinRelativeMoveActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_odin_relative_move",
      "ROS 2 relative move action name"),
    BT::InputPort<double>(
      "forward_m",
      0.0,
      "Positive forward, negative backward"),
    BT::InputPort<double>(
      "lateral_m",
      0.0,
      "Positive left, negative right"),
    BT::InputPort<double>(
      "timeout_sec",
      10.0,
      "Server-side movement timeout"),
    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Maximum time waiting for the action server"),
    BT::InputPort<int>(
      "result_grace_ms",
      2000,
      "Additional client-side result grace time"),
    BT::OutputPort<int>(
      "result_code",
      "Action result code; -1 means BT client-side failure"),
    BT::OutputPort<std::string>(
      "result_message",
      "Action result message"),
    BT::OutputPort<double>(
      "final_forward_error_m",
      "Final forward error"),
    BT::OutputPort<double>(
      "final_lateral_error_m",
      "Final lateral error")
  };
}

void R2OdinRelativeMoveActionNode::resetOutputs()
{
  setOutput("result_code", kClientFailureCode);
  setOutput("result_message", std::string());
  setOutput("final_forward_error_m", 0.0);
  setOutput("final_lateral_error_m", 0.0);
}

void R2OdinRelativeMoveActionNode::setFailureOutputs(
  const std::string & message)
{
  setOutput("result_code", kClientFailureCode);
  setOutput("result_message", message);
  setOutput("final_forward_error_m", 0.0);
  setOutput("final_lateral_error_m", 0.0);
}

void R2OdinRelativeMoveActionNode::cancelActiveGoal()
{
  if (!client_ || !goal_handle_) {
    return;
  }

  try {
    client_->async_cancel_goal(goal_handle_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeMoveActionNode] Cancel request failed: %s",
      e.what());
  }
}

BT::NodeStatus R2OdinRelativeMoveActionNode::onStart()
{
  resetOutputs();

  std::string action_name = "/r2_odin_relative_move";
  double forward_m = 0.0;
  double lateral_m = 0.0;
  double timeout_sec = 10.0;
  int server_timeout_ms = 3000;
  int result_grace_ms = 2000;

  getInput("action_name", action_name);
  getInput("forward_m", forward_m);
  getInput("lateral_m", lateral_m);
  getInput("timeout_sec", timeout_sec);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_grace_ms", result_grace_ms);

  if (
    action_name.empty() ||
    !std::isfinite(forward_m) ||
    !std::isfinite(lateral_m) ||
    !std::isfinite(timeout_sec) ||
    timeout_sec <= 0.0 ||
    server_timeout_ms <= 0 ||
    result_grace_ms < 0)
  {
    const std::string message = "invalid relative move BT input";
    setFailureOutputs(message);
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeMoveActionNode] %s",
      message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto timeout_ms =
    static_cast<long long>(std::ceil(timeout_sec * 1000.0));

  result_timeout_ = std::chrono::milliseconds(
    timeout_ms + result_grace_ms);

  start_time_ = std::chrono::steady_clock::now();

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;

    try {
      client_ = rclcpp_action::create_client<RelativeMove>(
        node_, action_name_);
    } catch (const std::exception & e) {
      setFailureOutputs(e.what());
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2OdinRelativeMoveActionNode] Failed to create client: %s",
        e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!client_->wait_for_action_server(
      std::chrono::milliseconds(server_timeout_ms)))
  {
    const std::string message =
      "relative move action server not available";
    setFailureOutputs(message);
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeMoveActionNode] %s: %s",
      message.c_str(),
      action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RelativeMove::Goal goal;
  goal.forward_m = forward_m;
  goal.lateral_m = lateral_m;
  goal.timeout_sec = timeout_sec;

  auto options =
    rclcpp_action::Client<RelativeMove>::SendGoalOptions();

  options.feedback_callback =
    [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const RelativeMove::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "[R2OdinRelativeMoveActionNode] stage=%u "
        "moved=(%.3f, %.3f) error=(%.3f, %.3f) "
        "cmd=(%.3f, %.3f, %.3f)",
        feedback->stage,
        feedback->moved_forward_m,
        feedback->moved_lateral_m,
        feedback->forward_error_m,
        feedback->lateral_error_m,
        feedback->command_vx_mps,
        feedback->command_vy_mps,
        feedback->command_wz_rad_s);
    };

  goal_handle_future_ = client_->async_send_goal(goal, options);
  goal_handle_.reset();
  stage_ = Stage::WAIT_GOAL_ACCEPTED;

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2OdinRelativeMoveActionNode] Goal sent: forward=%.3f lateral=%.3f",
    forward_m,
    lateral_m);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2OdinRelativeMoveActionNode::onRunning()
{
  if (isTimeout()) {
    cancelActiveGoal();

    const std::string message =
      "BT client timed out waiting for relative move result";
    setFailureOutputs(message);
    stage_ = Stage::IDLE;

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeMoveActionNode] %s",
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

    goal_handle_ = goal_handle_future_.get();

    if (!goal_handle_) {
      const std::string message =
        "relative move goal was rejected";
      setFailureOutputs(message);
      stage_ = Stage::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    result_future_ = client_->async_get_result(goal_handle_);
    stage_ = Stage::WAIT_RESULT;
    return BT::NodeStatus::RUNNING;
  }

  if (stage_ == Stage::WAIT_RESULT) {
    if (
      result_future_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }

    const auto wrapped = result_future_.get();
    stage_ = Stage::IDLE;

    if (!wrapped.result) {
      setFailureOutputs("relative move returned a null result");
      return BT::NodeStatus::FAILURE;
    }

    setOutput("result_code", static_cast<int>(wrapped.result->code));
    setOutput("result_message", wrapped.result->message);
    setOutput(
      "final_forward_error_m",
      wrapped.result->final_forward_error_m);
    setOutput(
      "final_lateral_error_m",
      wrapped.result->final_lateral_error_m);

    const bool success =
      wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result->code == RelativeMove::Result::SUCCESS;

    if (success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2OdinRelativeMoveActionNode] Success: %s",
        wrapped.result->message.c_str());
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinRelativeMoveActionNode] Failure: wrapped=%d code=%u message=%s",
      static_cast<int>(wrapped.code),
      wrapped.result->code,
      wrapped.result->message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}

void R2OdinRelativeMoveActionNode::onHalted()
{
  cancelActiveGoal();
  goal_handle_.reset();
  stage_ = Stage::IDLE;

  RCLCPP_WARN(
    node_->get_logger(),
    "[R2OdinRelativeMoveActionNode] Halted; cancel requested");
}

bool R2OdinRelativeMoveActionNode::isTimeout() const
{
  return (
    std::chrono::steady_clock::now() - start_time_) >
    result_timeout_;
}

}  // namespace r2_bt_nodes
