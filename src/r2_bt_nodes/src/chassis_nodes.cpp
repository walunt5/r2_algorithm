#include "r2_bt_nodes/chassis_nodes.hpp"

namespace r2_bt_nodes
{

R2ChassisStepCommandNode::R2ChassisStepCommandNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(15000))
{
}

BT::PortsList R2ChassisStepCommandNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_chassis/step_command",
      "ROS2 action name for chassis step command"),
    BT::InputPort<std::string>(
      "cmd_type",
      "MOVE_FLAT",
      "Step command: MOVE_FLAT, CLIMB_200, CLIMB_400, DESCEND_200, DESCEND_400"),
    BT::InputPort<std::string>(
      "from_block",
      "",
      "Current block id"),
    BT::InputPort<std::string>(
      "target_block",
      "",
      "Target block id"),
    BT::InputPort<int>(
      "current_h",
      0,
      "Current height in mm"),
    BT::InputPort<int>(
      "target_h",
      0,
      "Target height in mm"),
    BT::InputPort<std::string>(
      "edge_id",
      "",
      "Edge id"),
    BT::InputPort<double>(
      "timeout_sec",
      10.0,
      "STM32 task timeout in seconds"),
    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Timeout while waiting for action server"),
    BT::InputPort<int>(
      "result_timeout_ms",
      15000,
      "Timeout while waiting for action result")
  };
}

BT::NodeStatus R2ChassisStepCommandNode::onStart()
{
  std::string action_name = "/r2_chassis/step_command";
  std::string cmd_type = "MOVE_FLAT";
  std::string from_block = "";
  std::string target_block = "";
  int current_h = 0;
  int target_h = 0;
  std::string edge_id = "";
  double timeout_sec = 10.0;
  int server_timeout_ms = 3000;
  int result_timeout_ms = 15000;

  getInput("action_name", action_name);
  getInput("cmd_type", cmd_type);
  getInput("from_block", from_block);
  getInput("target_block", target_block);
  getInput("current_h", current_h);
  getInput("target_h", target_h);
  getInput("edge_id", edge_id);
  getInput("timeout_sec", timeout_sec);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_timeout_ms", result_timeout_ms);

  if (
    cmd_type != "MOVE_FLAT" &&
    cmd_type != "CLIMB_200" &&
    cmd_type != "CLIMB_400" &&
    cmd_type != "DESCEND_200" &&
    cmd_type != "DESCEND_400")
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ChassisStepCommandNode] Invalid cmd_type=%s. "
      "Valid values: MOVE_FLAT, CLIMB_200, CLIMB_400, DESCEND_200, DESCEND_400.",
      cmd_type.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (timeout_sec <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ChassisStepCommandNode] Invalid timeout_sec=%.2f. It must be > 0.",
      timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  if (result_timeout_ms <= 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ChassisStepCommandNode] Invalid result_timeout_ms=%d. It must be > 0.",
      result_timeout_ms);
    return BT::NodeStatus::FAILURE;
  }

  result_timeout_ = std::chrono::milliseconds(result_timeout_ms);
  start_time_ = std::chrono::steady_clock::now();

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<StepCommand>(node_, action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2ChassisStepCommandNode] Waiting for action server: %s",
    action_name_.c_str());

  const auto server_timeout = std::chrono::milliseconds(server_timeout_ms);

  if (!client_->wait_for_action_server(server_timeout)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ChassisStepCommandNode] Action server not available: %s",
      action_name_.c_str());

    stage_ = Stage::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = StepCommand::Goal();
  goal_msg.cmd_type = cmd_type;
  goal_msg.from_block = from_block;
  goal_msg.target_block = target_block;
  goal_msg.current_h = current_h;
  goal_msg.target_h = target_h;
  goal_msg.edge_id = edge_id;
  goal_msg.timeout_sec = static_cast<float>(timeout_sec);

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2ChassisStepCommandNode] Sending goal: cmd_type=%s from=%s target=%s current_h=%d target_h=%d edge_id=%s timeout_sec=%.2f",
    cmd_type.c_str(),
    from_block.c_str(),
    target_block.c_str(),
    current_h,
    target_h,
    edge_id.c_str(),
    timeout_sec);

  goal_handle_future_ = client_->async_send_goal(goal_msg);
  goal_handle_.reset();
  stage_ = Stage::WAIT_GOAL_ACCEPTED;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2ChassisStepCommandNode::onRunning()
{
  if (isTimeout()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ChassisStepCommandNode] Timeout while waiting for StepCommand result.");

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
        "[R2ChassisStepCommandNode] Goal was rejected by action server.");

      stage_ = Stage::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2ChassisStepCommandNode] Goal accepted, waiting for result...");

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
        "[R2ChassisStepCommandNode] Action did not succeed. result_code=%d",
        static_cast<int>(wrapped_result.code));

      return BT::NodeStatus::FAILURE;
    }

    const auto & result = wrapped_result.result;

    if (!result) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2ChassisStepCommandNode] Result is null.");

      return BT::NodeStatus::FAILURE;
    }

    if (result->success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2ChassisStepCommandNode] SUCCESS. final_state=%u error_code=%u message=%s",
        result->final_state,
        result->error_code,
        result->message.c_str());

      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ChassisStepCommandNode] FAILURE. final_state=%u error_code=%u message=%s",
      result->final_state,
      result->error_code,
      result->message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[R2ChassisStepCommandNode] Invalid internal stage.");

  return BT::NodeStatus::FAILURE;
}

void R2ChassisStepCommandNode::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[R2ChassisStepCommandNode] Halted.");

  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }

  stage_ = Stage::IDLE;
}

bool R2ChassisStepCommandNode::isTimeout() const
{
  const auto now = std::chrono::steady_clock::now();
  return (now - start_time_) > result_timeout_;
}

R2LiftControlNode::R2LiftControlNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(15000))
{
}

BT::PortsList R2LiftControlNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_chassis/lift_control",
      "ROS2 action name for chassis lift control"),
    BT::InputPort<int>(
      "target_h_mm",
      0,
      "Target lift height in millimeters"),
    BT::InputPort<int>(
      "mask",
      7,
      "Lift mask: bit0=lift1, bit1=lift2, bit2=lift3. 7 means all lifts"),
    BT::InputPort<double>(
      "timeout_sec",
      10.0,
      "STM32 task timeout in seconds"),
    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Timeout while waiting for action server"),
    BT::InputPort<int>(
      "result_timeout_ms",
      15000,
      "Timeout while waiting for action result")
  };
}

BT::NodeStatus R2LiftControlNode::onStart()
{
  std::string action_name = "/r2_chassis/lift_control";
  int target_h_mm = 0;
  int mask = 7;
  double timeout_sec = 10.0;
  int server_timeout_ms = 3000;
  int result_timeout_ms = 15000;

  getInput("action_name", action_name);
  getInput("target_h_mm", target_h_mm);
  getInput("mask", mask);
  getInput("timeout_sec", timeout_sec);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_timeout_ms", result_timeout_ms);

  if (mask == 0 || (mask & ~0x07)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2LiftControlNode] Invalid mask=0x%02X. Valid values: 1, 2, 4, 7.",
      mask);
    return BT::NodeStatus::FAILURE;
  }

  if (timeout_sec <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2LiftControlNode] Invalid timeout_sec=%.2f. It must be > 0.",
      timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  if (result_timeout_ms <= 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2LiftControlNode] Invalid result_timeout_ms=%d. It must be > 0.",
      result_timeout_ms);
    return BT::NodeStatus::FAILURE;
  }

  result_timeout_ = std::chrono::milliseconds(result_timeout_ms);
  start_time_ = std::chrono::steady_clock::now();

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<LiftControl>(node_, action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2LiftControlNode] Waiting for action server: %s",
    action_name_.c_str());

  const auto server_timeout = std::chrono::milliseconds(server_timeout_ms);

  if (!client_->wait_for_action_server(server_timeout)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2LiftControlNode] Action server not available: %s",
      action_name_.c_str());

    stage_ = Stage::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = LiftControl::Goal();
  goal_msg.target_h_mm = target_h_mm;
  goal_msg.mask = static_cast<uint8_t>(mask);
  goal_msg.timeout_sec = static_cast<float>(timeout_sec);

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2LiftControlNode] Sending goal: target_h_mm=%d mask=0x%02X timeout_sec=%.2f",
    target_h_mm,
    mask,
    timeout_sec);

  goal_handle_future_ = client_->async_send_goal(goal_msg);
  goal_handle_.reset();
  stage_ = Stage::WAIT_GOAL_ACCEPTED;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2LiftControlNode::onRunning()
{
  if (isTimeout()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2LiftControlNode] Timeout while waiting for LiftControl result.");

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
        "[R2LiftControlNode] Goal was rejected by action server.");

      stage_ = Stage::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2LiftControlNode] Goal accepted, waiting for result...");

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
        "[R2LiftControlNode] Action did not succeed. result_code=%d",
        static_cast<int>(wrapped_result.code));

      return BT::NodeStatus::FAILURE;
    }

    const auto & result = wrapped_result.result;

    if (!result) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2LiftControlNode] Result is null.");

      return BT::NodeStatus::FAILURE;
    }

    if (result->success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2LiftControlNode] SUCCESS. final_state=%u error_code=%u message=%s",
        result->final_state,
        result->error_code,
        result->message.c_str());

      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2LiftControlNode] FAILURE. final_state=%u error_code=%u message=%s",
      result->final_state,
      result->error_code,
      result->message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[R2LiftControlNode] Invalid internal stage.");

  return BT::NodeStatus::FAILURE;
}

void R2LiftControlNode::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[R2LiftControlNode] Halted.");

  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }

  stage_ = Stage::IDLE;
}

bool R2LiftControlNode::isTimeout() const
{
  const auto now = std::chrono::steady_clock::now();
  return (now - start_time_) > result_timeout_;
}

}  // namespace r2_bt_nodes