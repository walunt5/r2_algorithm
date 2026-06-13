#include "r2_bt_nodes/odin_nodes.hpp"

namespace r2_bt_nodes
{

R2OdinPosePidAlignActionNode::R2OdinPosePidAlignActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(8000))
{
}

BT::PortsList R2OdinPosePidAlignActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_odin_pose_pid/align",
      "ROS2 action name for Odin pose PID align"),
    BT::InputPort<std::string>(
      "goal_name",
      "",
      "Preset goal name for alignment"),
    BT::InputPort<bool>(
      "use_goal_name",
      true,
      "If true, server resolves goal_name from YAML"),
    BT::InputPort<double>(
      "timeout_sec",
      5.0,
      "Align timeout in seconds"),
    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Timeout while waiting for action server"),
    BT::InputPort<int>(
      "result_timeout_ms",
      8000,
      "Timeout while waiting for action result")
  };
}

BT::NodeStatus R2OdinPosePidAlignActionNode::onStart()
{
  std::string action_name = "/r2_odin_pose_pid/align";
  std::string goal_name = "";
  bool use_goal_name = true;
  double timeout_sec = 5.0;
  int server_timeout_ms = 3000;
  int result_timeout_ms = 8000;

  getInput("action_name", action_name);
  getInput("goal_name", goal_name);
  getInput("use_goal_name", use_goal_name);
  getInput("timeout_sec", timeout_sec);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_timeout_ms", result_timeout_ms);

  if (use_goal_name && goal_name.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinPosePidAlignActionNode] goal_name is empty while use_goal_name=true");
    return BT::NodeStatus::FAILURE;
  }

  if (timeout_sec <= 0.0 || server_timeout_ms <= 0 || result_timeout_ms <= 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinPosePidAlignActionNode] Invalid timeout config");
    return BT::NodeStatus::FAILURE;
  }

  result_timeout_ = std::chrono::milliseconds(result_timeout_ms);
  start_time_ = std::chrono::steady_clock::now();

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<OdinPosePidAlign>(node_, action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2OdinPosePidAlignActionNode] Waiting for action server: %s",
    action_name_.c_str());

  if (!client_->wait_for_action_server(std::chrono::milliseconds(server_timeout_ms))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinPosePidAlignActionNode] Action server not available: %s",
      action_name_.c_str());
    stage_ = Stage::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = OdinPosePidAlign::Goal();
  goal_msg.goal_name = goal_name;
  goal_msg.use_goal_name = use_goal_name;
  goal_msg.timeout_sec = static_cast<float>(timeout_sec);

  goal_msg.target_pose.header.stamp = node_->get_clock()->now();
  goal_msg.target_pose.header.frame_id = "map";
  goal_msg.target_pose.pose.orientation.w = 1.0;

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2OdinPosePidAlignActionNode] Sending align goal: goal_name=%s timeout_sec=%.2f",
    goal_name.c_str(),
    timeout_sec);

  auto send_goal_options =
    rclcpp_action::Client<OdinPosePidAlign>::SendGoalOptions();

  send_goal_options.feedback_callback =
    [this](
      GoalHandleAlign::SharedPtr,
      const std::shared_ptr<const OdinPosePidAlign::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "[R2OdinPosePidAlignActionNode] feedback state=%s err=(%.3f, %.3f, %.3f) cmd=(%.3f, %.3f, %.3f)",
        feedback->state.c_str(),
        feedback->error_x,
        feedback->error_y,
        feedback->error_yaw,
        feedback->cmd_vx,
        feedback->cmd_vy,
        feedback->cmd_wz);
    };

    goal_handle_future_ =
        client_->async_send_goal(goal_msg, send_goal_options);

  stage_ = Stage::WAIT_GOAL_ACCEPTED;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2OdinPosePidAlignActionNode::onRunning()
{
  if (isTimeout()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2OdinPosePidAlignActionNode] Timeout while waiting for result");

    if (goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
    }

    stage_ = Stage::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  if (stage_ == Stage::WAIT_GOAL_ACCEPTED) {
    const auto status = goal_handle_future_.wait_for(std::chrono::milliseconds(0));

    if (status != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    goal_handle_ = goal_handle_future_.get();

    if (!goal_handle_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2OdinPosePidAlignActionNode] Goal was rejected by action server");
      stage_ = Stage::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2OdinPosePidAlignActionNode] Goal accepted, waiting for result...");

    result_future_ = client_->async_get_result(goal_handle_);
    stage_ = Stage::WAIT_RESULT;
    return BT::NodeStatus::RUNNING;
  }

  if (stage_ == Stage::WAIT_RESULT) {
    const auto status = result_future_.wait_for(std::chrono::milliseconds(0));

    if (status != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    const auto wrapped_result = result_future_.get();
    stage_ = Stage::IDLE;

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2OdinPosePidAlignActionNode] Action did not succeed. result_code=%d",
        static_cast<int>(wrapped_result.code));
      return BT::NodeStatus::FAILURE;
    }

    if (!wrapped_result.result->success) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2OdinPosePidAlignActionNode] Align failed: %s",
        wrapped_result.result->message.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2OdinPosePidAlignActionNode] Align succeeded: %s final_error=(%.3f, %.3f, %.3f)",
      wrapped_result.result->message.c_str(),
      wrapped_result.result->final_error_x,
      wrapped_result.result->final_error_y,
      wrapped_result.result->final_error_yaw);

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void R2OdinPosePidAlignActionNode::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[R2OdinPosePidAlignActionNode] Halted, canceling goal if active");

  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }

  stage_ = Stage::IDLE;
}

bool R2OdinPosePidAlignActionNode::isTimeout() const
{
  const auto now = std::chrono::steady_clock::now();
  return (now - start_time_) > result_timeout_;
}

}  // namespace r2_bt_nodes