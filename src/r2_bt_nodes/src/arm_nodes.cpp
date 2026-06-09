#include "r2_bt_nodes/arm_nodes.hpp"

namespace r2_bt_nodes
{

R2GetHeadActionNode::R2GetHeadActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(12000))
{
}

BT::PortsList R2GetHeadActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_arm/get_head",
      "ROS2 action name for GetHead"),
    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Timeout while waiting for action server"),
    BT::InputPort<int>(
      "result_timeout_ms",
      12000,
      "Timeout while waiting for action result")
  };
}

BT::NodeStatus R2GetHeadActionNode::onStart()
{
  std::string action_name = "/r2_arm/get_head";
  int server_timeout_ms = 3000;
  int result_timeout_ms = 12000;

  getInput("action_name", action_name);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_timeout_ms", result_timeout_ms);

  result_timeout_ = std::chrono::milliseconds(result_timeout_ms);
  start_time_ = std::chrono::steady_clock::now();

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<GetHead>(node_, action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2GetHeadActionNode] Waiting for action server: %s",
    action_name_.c_str());

  const auto server_timeout = std::chrono::milliseconds(server_timeout_ms);

  if (!client_->wait_for_action_server(server_timeout)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2GetHeadActionNode] Action server not available: %s",
      action_name_.c_str());

    stage_ = Stage::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = GetHead::Goal();

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2GetHeadActionNode] Sending GetHead goal...");

  goal_handle_future_ = client_->async_send_goal(goal_msg);
  goal_handle_.reset();
  stage_ = Stage::WAIT_GOAL_ACCEPTED;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2GetHeadActionNode::onRunning()
{
  if (isTimeout()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2GetHeadActionNode] Timeout while waiting for GetHead result.");

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
        "[R2GetHeadActionNode] Goal was rejected by action server.");

      stage_ = Stage::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2GetHeadActionNode] Goal accepted, waiting for result...");

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
        "[R2GetHeadActionNode] Action did not succeed. result_code=%d",
        static_cast<int>(wrapped_result.code));

      return BT::NodeStatus::FAILURE;
    }

    const auto & result = wrapped_result.result;

    if (!result) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2GetHeadActionNode] Result is null.");

      return BT::NodeStatus::FAILURE;
    }

    if (result->success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2GetHeadActionNode] SUCCESS. final_state=%u error_code=%u message=%s",
        result->final_state,
        result->error_code,
        result->message.c_str());

      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2GetHeadActionNode] FAILURE. final_state=%u error_code=%u message=%s",
      result->final_state,
      result->error_code,
      result->message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[R2GetHeadActionNode] Invalid internal stage.");

  return BT::NodeStatus::FAILURE;
}

void R2GetHeadActionNode::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[R2GetHeadActionNode] Halted.");

  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }

  stage_ = Stage::IDLE;
}

bool R2GetHeadActionNode::isTimeout() const
{
  const auto now = std::chrono::steady_clock::now();
  return (now - start_time_) > result_timeout_;
}


R2SetEndEffectorNode::R2SetEndEffectorNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(3000))
{
}

BT::PortsList R2SetEndEffectorNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_arm/set_end_effector",
      "ROS2 action name for SetEndEffector"),
    BT::InputPort<int>(
      "io_id",
      1,
      "End effector IO id: 1=gripper claw, 2=arm suction pump, 3=rear storage pump"),
    BT::InputPort<int>(
      "state",
      0,
      "End effector state: 0=off/open, 1=on/close"),
    BT::InputPort<int>(
      "hold_ms",
      0,
      "Hold time in milliseconds"),
    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Timeout while waiting for action server"),
    BT::InputPort<int>(
      "result_timeout_ms",
      3000,
      "Timeout while waiting for action result")
  };
}

BT::NodeStatus R2SetEndEffectorNode::onStart()
{
  std::string action_name = "/r2_arm/set_end_effector";
  int io_id = 1;
  int state = 0;
  int hold_ms = 0;
  int server_timeout_ms = 3000;
  int result_timeout_ms = 3000;

  getInput("action_name", action_name);
  getInput("io_id", io_id);
  getInput("state", state);
  getInput("hold_ms", hold_ms);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_timeout_ms", result_timeout_ms);

  if (io_id < 1 || io_id > 3) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2SetEndEffectorNode] Invalid io_id=%d. Valid values: 1, 2, 3.",
      io_id);
    return BT::NodeStatus::FAILURE;
  }

  if (state < 0 || state > 1) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2SetEndEffectorNode] Invalid state=%d. Valid values: 0, 1.",
      state);
    return BT::NodeStatus::FAILURE;
  }

  if (hold_ms < 0 || hold_ms > 65535) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2SetEndEffectorNode] Invalid hold_ms=%d. Valid range: 0~65535.",
      hold_ms);
    return BT::NodeStatus::FAILURE;
  }

  result_timeout_ = std::chrono::milliseconds(result_timeout_ms);
  start_time_ = std::chrono::steady_clock::now();

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<SetEndEffector>(node_, action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2SetEndEffectorNode] Waiting for action server: %s",
    action_name_.c_str());

  const auto server_timeout = std::chrono::milliseconds(server_timeout_ms);

  if (!client_->wait_for_action_server(server_timeout)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2SetEndEffectorNode] Action server not available: %s",
      action_name_.c_str());

    stage_ = Stage::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = SetEndEffector::Goal();
  goal_msg.io_id = static_cast<uint8_t>(io_id);
  goal_msg.state = static_cast<uint8_t>(state);
  goal_msg.hold_ms = static_cast<uint16_t>(hold_ms);

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2SetEndEffectorNode] Sending goal: io_id=%d state=%d hold_ms=%d",
    io_id,
    state,
    hold_ms);

  goal_handle_future_ = client_->async_send_goal(goal_msg);
  goal_handle_.reset();
  stage_ = Stage::WAIT_GOAL_ACCEPTED;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2SetEndEffectorNode::onRunning()
{
  if (isTimeout()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2SetEndEffectorNode] Timeout while waiting for SetEndEffector result.");

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
        "[R2SetEndEffectorNode] Goal was rejected by action server.");

      stage_ = Stage::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2SetEndEffectorNode] Goal accepted, waiting for result...");

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
        "[R2SetEndEffectorNode] Action did not succeed. result_code=%d",
        static_cast<int>(wrapped_result.code));

      return BT::NodeStatus::FAILURE;
    }

    const auto & result = wrapped_result.result;

    if (!result) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2SetEndEffectorNode] Result is null.");

      return BT::NodeStatus::FAILURE;
    }

    if (result->success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2SetEndEffectorNode] SUCCESS. final_state=%u error_code=%u feedback_ok=%s message=%s",
        result->final_state,
        result->error_code,
        result->feedback_ok ? "true" : "false",
        result->message.c_str());

      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2SetEndEffectorNode] FAILURE. final_state=%u error_code=%u feedback_ok=%s message=%s",
      result->final_state,
      result->error_code,
      result->feedback_ok ? "true" : "false",
      result->message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[R2SetEndEffectorNode] Invalid internal stage.");

  return BT::NodeStatus::FAILURE;
}

void R2SetEndEffectorNode::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[R2SetEndEffectorNode] Halted.");

  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }

  stage_ = Stage::IDLE;
}

bool R2SetEndEffectorNode::isTimeout() const
{
  const auto now = std::chrono::steady_clock::now();
  return (now - start_time_) > result_timeout_;
}

R2ExecuteArmActionNode::R2ExecuteArmActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(10000))
{
}

BT::PortsList R2ExecuteArmActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_arm/execute_action",
      "ROS2 action name for ExecuteAction"),
    BT::InputPort<int>(
      "target_id",
      2,
      "Target id: 1=gripper arm, 2=KFS suction arm, 3=composite task"),
    BT::InputPort<int>(
      "action_id",
      0x0201,
      "Action id defined by STM32 protocol"),
    BT::InputPort<int>(
      "timeout_ms",
      8000,
      "STM32 action timeout in milliseconds"),
    BT::InputPort<int>(
      "param",
      0,
      "Optional int16 parameter"),
    BT::InputPort<int>(
      "flags",
      0,
      "Optional uint8 flags"),
    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Timeout while waiting for action server"),
    BT::InputPort<int>(
      "result_timeout_ms",
      10000,
      "Timeout while waiting for action result")
  };
}

BT::NodeStatus R2ExecuteArmActionNode::onStart()
{
  std::string action_name = "/r2_arm/execute_action";
  int target_id = 2;
  int action_id = 0x0201;
  int timeout_ms = 8000;
  int param = 0;
  int flags = 0;
  int server_timeout_ms = 3000;
  int result_timeout_ms = 10000;

  getInput("action_name", action_name);
  getInput("target_id", target_id);
  getInput("action_id", action_id);
  getInput("timeout_ms", timeout_ms);
  getInput("param", param);
  getInput("flags", flags);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_timeout_ms", result_timeout_ms);

  if (target_id < 1 || target_id > 3) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ExecuteArmActionNode] Invalid target_id=%d. Valid values: 1, 2, 3.",
      target_id);
    return BT::NodeStatus::FAILURE;
  }

  if (action_id < 0 || action_id > 65535) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ExecuteArmActionNode] Invalid action_id=%d. Valid range: 0~65535.",
      action_id);
    return BT::NodeStatus::FAILURE;
  }

  if (timeout_ms < 0 || timeout_ms > 65535) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ExecuteArmActionNode] Invalid timeout_ms=%d. Valid range: 0~65535.",
      timeout_ms);
    return BT::NodeStatus::FAILURE;
  }

  if (param < -32768 || param > 32767) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ExecuteArmActionNode] Invalid param=%d. Valid range: -32768~32767.",
      param);
    return BT::NodeStatus::FAILURE;
  }

  if (flags < 0 || flags > 255) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ExecuteArmActionNode] Invalid flags=%d. Valid range: 0~255.",
      flags);
    return BT::NodeStatus::FAILURE;
  }

  result_timeout_ = std::chrono::milliseconds(result_timeout_ms);
  start_time_ = std::chrono::steady_clock::now();

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<ExecuteAction>(node_, action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2ExecuteArmActionNode] Waiting for action server: %s",
    action_name_.c_str());

  const auto server_timeout = std::chrono::milliseconds(server_timeout_ms);

  if (!client_->wait_for_action_server(server_timeout)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ExecuteArmActionNode] Action server not available: %s",
      action_name_.c_str());

    stage_ = Stage::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = ExecuteAction::Goal();
  goal_msg.target_id = static_cast<uint8_t>(target_id);
  goal_msg.action_id = static_cast<uint16_t>(action_id);
  goal_msg.timeout_ms = static_cast<uint16_t>(timeout_ms);
  goal_msg.param = static_cast<int16_t>(param);
  goal_msg.flags = static_cast<uint8_t>(flags);

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2ExecuteArmActionNode] Sending goal: target_id=%d action_id=0x%04X timeout_ms=%d param=%d flags=0x%02X",
    target_id,
    action_id,
    timeout_ms,
    param,
    flags);

  goal_handle_future_ = client_->async_send_goal(goal_msg);
  goal_handle_.reset();
  stage_ = Stage::WAIT_GOAL_ACCEPTED;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2ExecuteArmActionNode::onRunning()
{
  if (isTimeout()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ExecuteArmActionNode] Timeout while waiting for ExecuteAction result.");

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
        "[R2ExecuteArmActionNode] Goal was rejected by action server.");

      stage_ = Stage::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2ExecuteArmActionNode] Goal accepted, waiting for result...");

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
        "[R2ExecuteArmActionNode] Action did not succeed. result_code=%d",
        static_cast<int>(wrapped_result.code));

      return BT::NodeStatus::FAILURE;
    }

    const auto & result = wrapped_result.result;

    if (!result) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2ExecuteArmActionNode] Result is null.");

      return BT::NodeStatus::FAILURE;
    }

    if (result->success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2ExecuteArmActionNode] SUCCESS. final_state=%u error_code=%u message=%s",
        result->final_state,
        result->error_code,
        result->message.c_str());

      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2ExecuteArmActionNode] FAILURE. final_state=%u error_code=%u message=%s",
      result->final_state,
      result->error_code,
      result->message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[R2ExecuteArmActionNode] Invalid internal stage.");

  return BT::NodeStatus::FAILURE;
}

void R2ExecuteArmActionNode::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[R2ExecuteArmActionNode] Halted.");

  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }

  stage_ = Stage::IDLE;
}

bool R2ExecuteArmActionNode::isTimeout() const
{
  const auto now = std::chrono::steady_clock::now();
  return (now - start_time_) > result_timeout_;
}

}  // namespace r2_bt_nodes