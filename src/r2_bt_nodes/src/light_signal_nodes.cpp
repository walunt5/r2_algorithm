#include "r2_bt_nodes/light_signal_nodes.hpp"

#include <cmath>

namespace r2_bt_nodes
{

R2WaitForLightSignalActionNode::R2WaitForLightSignalActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(5000)),
  request_generation_(0)
{
}

BT::PortsList R2WaitForLightSignalActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name", "/r2_light_signal/wait", "ROS 2 light signal wait action name"),
    BT::InputPort<double>(
      "timeout_sec", 3.0, "Maximum time to wait for a new true signal"),
    BT::InputPort<int>(
      "server_timeout_ms", 3000, "Timeout while waiting for the action server"),
    BT::InputPort<int>(
      "result_grace_ms", 2000, "Client grace period beyond the server timeout")
  };
}

BT::NodeStatus R2WaitForLightSignalActionNode::onStart()
{
  std::string action_name = "/r2_light_signal/wait";
  double timeout_sec = 3.0;
  int server_timeout_ms = 3000;
  int result_grace_ms = 2000;
  getInput("action_name", action_name);
  getInput("timeout_sec", timeout_sec);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_grace_ms", result_grace_ms);

  if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0 ||
    server_timeout_ms <= 0 || result_grace_ms <= 0)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WaitForLightSignalActionNode] Invalid timeout configuration");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<WaitForLightSignal>(node_, action_name_);
  }
  if (!client_->wait_for_action_server(std::chrono::milliseconds(server_timeout_ms))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WaitForLightSignalActionNode] Action server unavailable: %s",
      action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto timeout_ms = static_cast<std::int64_t>(std::ceil(timeout_sec * 1000.0));
  result_timeout_ = std::chrono::milliseconds(timeout_ms + result_grace_ms);
  start_time_ = std::chrono::steady_clock::now();
  goal_handle_.reset();

  WaitForLightSignal::Goal goal;
  goal.timeout_sec = static_cast<float>(timeout_sec);
  const std::uint64_t generation = request_generation_.fetch_add(1) + 1;

  auto options = rclcpp_action::Client<WaitForLightSignal>::SendGoalOptions();
  options.goal_response_callback =
    [this, generation](GoalHandle::SharedPtr goal_handle) {
      if (goal_handle && request_generation_.load() != generation) {
        client_->async_cancel_goal(goal_handle);
      }
    };
  options.feedback_callback =
    [this](
    GoalHandle::SharedPtr,
    const std::shared_ptr<const WaitForLightSignal::Feedback> feedback)
    {
      if (!feedback) {
        return;
      }
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[R2WaitForLightSignalActionNode] feedback: received=%s detected=%s elapsed=%.2f",
        feedback->signal_received ? "true" : "false",
        feedback->detected ? "true" : "false",
        feedback->elapsed_sec);
    };

  goal_handle_future_ = client_->async_send_goal(goal, options);
  stage_ = Stage::WAIT_GOAL_ACCEPTED;
  RCLCPP_INFO(
    node_->get_logger(),
    "[R2WaitForLightSignalActionNode] Waiting up to %.3fs for a new true signal",
    timeout_sec);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2WaitForLightSignalActionNode::onRunning()
{
  if (isTimeout()) {
    request_generation_.fetch_add(1);
    if (goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
    }
    stage_ = Stage::IDLE;
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WaitForLightSignalActionNode] Client result timeout");
    return BT::NodeStatus::FAILURE;
  }

  if (stage_ == Stage::WAIT_GOAL_ACCEPTED) {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      stage_ = Stage::IDLE;
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2WaitForLightSignalActionNode] Goal was rejected");
      return BT::NodeStatus::FAILURE;
    }
    result_future_ = client_->async_get_result(goal_handle_);
    stage_ = Stage::WAIT_RESULT;
    return BT::NodeStatus::RUNNING;
  }

  if (stage_ == Stage::WAIT_RESULT) {
    if (result_future_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    const auto wrapped_result = result_future_.get();
    stage_ = Stage::IDLE;
    const auto & result = wrapped_result.result;
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result && result->success)
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2WaitForLightSignalActionNode] SUCCESS: %s",
        result->message.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2WaitForLightSignalActionNode] FAILURE: code=%d message=%s",
      static_cast<int>(wrapped_result.code),
      result ? result->message.c_str() : "null result");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[R2WaitForLightSignalActionNode] Invalid internal stage");
  return BT::NodeStatus::FAILURE;
}

void R2WaitForLightSignalActionNode::onHalted()
{
  request_generation_.fetch_add(1);
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  stage_ = Stage::IDLE;
  RCLCPP_WARN(node_->get_logger(), "[R2WaitForLightSignalActionNode] Halted");
}

bool R2WaitForLightSignalActionNode::isTimeout() const
{
  return (std::chrono::steady_clock::now() - start_time_) > result_timeout_;
}

}  // namespace r2_bt_nodes
