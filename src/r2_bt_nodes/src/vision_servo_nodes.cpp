#include "r2_bt_nodes/vision_servo_nodes.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>

namespace r2_bt_nodes
{
namespace
{

using VisionServo = r2_vision_servo_interfaces::action::VisionServo;

std::string lowerString(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

bool parseAlignStrategy(const std::string & text, uint8_t & out)
{
  const auto value = lowerString(text);
  if (value == "1" || value == "xy_parallel") {
    out = VisionServo::Goal::ALIGN_STRATEGY_XY_PARALLEL;
    return true;
  }
  if (value == "2" || value == "y_then_x") {
    out = VisionServo::Goal::ALIGN_STRATEGY_Y_THEN_X;
    return true;
  }
  if (value == "3" || value == "yaw_then_y_then_x") {
    out = VisionServo::Goal::ALIGN_STRATEGY_YAW_THEN_Y_THEN_X;
    return true;
  }
  if (value == "4" || value == "yaw_gate_xy_parallel") {
    out = VisionServo::Goal::ALIGN_STRATEGY_YAW_GATE_XY_PARALLEL;
    return true;
  }
  return false;
}

bool parseYawMode(const std::string & text, uint8_t & out)
{
  const auto value = lowerString(text);
  if (value == "0" || value == "none") {
    out = VisionServo::Goal::YAW_MODE_NONE;
    return true;
  }
  if (value == "1" || value == "hold_current_odin_yaw" || value == "hold_current_yaw") {
    out = VisionServo::Goal::YAW_MODE_HOLD_CURRENT_ODIN_YAW;
    return true;
  }
  if (value == "2" || value == "use_goal_yaw") {
    out = VisionServo::Goal::YAW_MODE_USE_GOAL_YAW;
    return true;
  }
  return false;
}

bool validUint8(int value)
{
  return value >= 0 && value <= 255;
}

}  // namespace

R2VisionServoActionNode::R2VisionServoActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(8000))
{
}

BT::PortsList R2VisionServoActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_chassis/vision_servo",
      "ROS2 action name for vision servo"),
    BT::InputPort<int>("target_type", 1, "Vision target type"),
    BT::InputPort<int>("zone_id", 1, "Vision zone id"),
    BT::InputPort<bool>("use_class_id", true, "Use exact class_id filter"),
    BT::InputPort<int>("class_id", 100, "Vision class id"),
    BT::InputPort<bool>("use_color", false, "Use exact color filter"),
    BT::InputPort<int>("color", 0, "Vision color id"),
    BT::InputPort<double>("min_confidence", 0.5, "Minimum vision confidence"),
    BT::InputPort<double>("max_frame_age_sec", 0.20, "Maximum vision frame age"),
    BT::InputPort<double>("desired_robot_x", 0.30, "Desired target x in robot frame"),
    BT::InputPort<double>("desired_robot_y", 0.0, "Desired target y in robot frame"),
    BT::InputPort<double>("desired_robot_z", 0.0, "Desired target z in robot frame"),
    BT::InputPort<double>("tolerance_x", 0.03, "Tolerance for robot x"),
    BT::InputPort<double>("tolerance_y", 0.02, "Tolerance for robot y"),
    BT::InputPort<double>("tolerance_z", 999.0, "Tolerance for robot z"),
    BT::InputPort<std::string>(
      "align_strategy",
      "yaw_then_y_then_x",
      "Alignment strategy"),
    BT::InputPort<std::string>(
      "yaw_mode",
      "hold_current_odin_yaw",
      "Yaw control mode"),
    BT::InputPort<double>("target_yaw_rad", 0.0, "Target yaw when yaw_mode=use_goal_yaw"),
    BT::InputPort<double>("yaw_tolerance_rad", 0.035, "Yaw success tolerance"),
    BT::InputPort<double>("yaw_gate_rad", 0.10, "Yaw gate threshold"),
    BT::InputPort<int>("timeout_ms", 6000, "Vision servo timeout"),
    BT::InputPort<int>("stable_required_frames", 5, "Stable frames required"),
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

BT::NodeStatus R2VisionServoActionNode::onStart()
{
  std::string action_name = "/r2_chassis/vision_servo";
  int target_type = 1;
  int zone_id = 1;
  bool use_class_id = true;
  int class_id = 100;
  bool use_color = false;
  int color = 0;
  double min_confidence = 0.5;
  double max_frame_age_sec = 0.20;
  double desired_robot_x = 0.30;
  double desired_robot_y = 0.0;
  double desired_robot_z = 0.0;
  double tolerance_x = 0.03;
  double tolerance_y = 0.02;
  double tolerance_z = 999.0;
  std::string align_strategy_text = "yaw_then_y_then_x";
  std::string yaw_mode_text = "hold_current_odin_yaw";
  double target_yaw_rad = 0.0;
  double yaw_tolerance_rad = 0.035;
  double yaw_gate_rad = 0.10;
  int timeout_ms = 6000;
  int stable_required_frames = 5;
  int server_timeout_ms = 3000;
  int result_timeout_ms = 8000;

  getInput("action_name", action_name);
  getInput("target_type", target_type);
  getInput("zone_id", zone_id);
  getInput("use_class_id", use_class_id);
  getInput("class_id", class_id);
  getInput("use_color", use_color);
  getInput("color", color);
  getInput("min_confidence", min_confidence);
  getInput("max_frame_age_sec", max_frame_age_sec);
  getInput("desired_robot_x", desired_robot_x);
  getInput("desired_robot_y", desired_robot_y);
  getInput("desired_robot_z", desired_robot_z);
  getInput("tolerance_x", tolerance_x);
  getInput("tolerance_y", tolerance_y);
  getInput("tolerance_z", tolerance_z);
  getInput("align_strategy", align_strategy_text);
  getInput("yaw_mode", yaw_mode_text);
  getInput("target_yaw_rad", target_yaw_rad);
  getInput("yaw_tolerance_rad", yaw_tolerance_rad);
  getInput("yaw_gate_rad", yaw_gate_rad);
  getInput("timeout_ms", timeout_ms);
  getInput("stable_required_frames", stable_required_frames);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_timeout_ms", result_timeout_ms);

  uint8_t align_strategy = VisionServo::Goal::ALIGN_STRATEGY_YAW_THEN_Y_THEN_X;
  uint8_t yaw_mode = VisionServo::Goal::YAW_MODE_HOLD_CURRENT_ODIN_YAW;
  if (!parseAlignStrategy(align_strategy_text, align_strategy)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2VisionServoActionNode] Invalid align_strategy=%s",
      align_strategy_text.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!parseYawMode(yaw_mode_text, yaw_mode)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2VisionServoActionNode] Invalid yaw_mode=%s",
      yaw_mode_text.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!validUint8(target_type) || !validUint8(zone_id) || !validUint8(class_id) ||
    !validUint8(color))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2VisionServoActionNode] target_type/zone_id/class_id/color must be 0~255");
    return BT::NodeStatus::FAILURE;
  }
  if (timeout_ms <= 0 || stable_required_frames <= 0 || stable_required_frames > 255 ||
    server_timeout_ms <= 0 || result_timeout_ms <= 0 || max_frame_age_sec <= 0.0)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2VisionServoActionNode] Invalid timeout/stable/frame-age config");
    return BT::NodeStatus::FAILURE;
  }

  result_timeout_ = std::chrono::milliseconds(result_timeout_ms);
  start_time_ = std::chrono::steady_clock::now();

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<VisionServo>(node_, action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2VisionServoActionNode] Waiting for action server: %s",
    action_name_.c_str());

  if (!client_->wait_for_action_server(std::chrono::milliseconds(server_timeout_ms))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2VisionServoActionNode] Action server not available: %s",
      action_name_.c_str());
    stage_ = Stage::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = VisionServo::Goal();
  goal_msg.target_type = static_cast<uint8_t>(target_type);
  goal_msg.zone_id = static_cast<uint8_t>(zone_id);
  goal_msg.use_class_id = use_class_id;
  goal_msg.class_id = static_cast<uint8_t>(class_id);
  goal_msg.use_color = use_color;
  goal_msg.color = static_cast<uint8_t>(color);
  goal_msg.min_confidence = static_cast<float>(min_confidence);
  goal_msg.max_frame_age_sec = static_cast<float>(max_frame_age_sec);
  goal_msg.desired_robot_x = static_cast<float>(desired_robot_x);
  goal_msg.desired_robot_y = static_cast<float>(desired_robot_y);
  goal_msg.desired_robot_z = static_cast<float>(desired_robot_z);
  goal_msg.tolerance_x = static_cast<float>(tolerance_x);
  goal_msg.tolerance_y = static_cast<float>(tolerance_y);
  goal_msg.tolerance_z = static_cast<float>(tolerance_z);
  goal_msg.align_strategy = align_strategy;
  goal_msg.yaw_mode = yaw_mode;
  goal_msg.target_yaw_rad = static_cast<float>(target_yaw_rad);
  goal_msg.yaw_tolerance_rad = static_cast<float>(yaw_tolerance_rad);
  goal_msg.yaw_gate_rad = static_cast<float>(yaw_gate_rad);
  goal_msg.timeout_ms = static_cast<uint32_t>(timeout_ms);
  goal_msg.stable_required_frames = static_cast<uint8_t>(stable_required_frames);

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2VisionServoActionNode] Sending goal: type=%d zone=%d class_id=%d desired=(%.3f, %.3f, %.3f) strategy=%s yaw_mode=%s timeout_ms=%d",
    target_type,
    zone_id,
    class_id,
    desired_robot_x,
    desired_robot_y,
    desired_robot_z,
    align_strategy_text.c_str(),
    yaw_mode_text.c_str(),
    timeout_ms);

  auto send_goal_options =
    rclcpp_action::Client<VisionServo>::SendGoalOptions();

  send_goal_options.feedback_callback =
    [this](
      GoalHandleVisionServo::SharedPtr,
      const std::shared_ptr<const VisionServo::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "[R2VisionServoActionNode] feedback phase=%u err=(%.3f, %.3f, %.3f) yaw_err=%.3f cmd=(%.3f, %.3f, %.3f) %s",
        feedback->phase,
        feedback->error_x,
        feedback->error_y,
        feedback->error_z,
        feedback->yaw_error_rad,
        feedback->cmd_vx,
        feedback->cmd_vy,
        feedback->cmd_wz,
        feedback->message.c_str());
    };

  goal_handle_future_ = client_->async_send_goal(goal_msg, send_goal_options);
  stage_ = Stage::WAIT_GOAL_ACCEPTED;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2VisionServoActionNode::onRunning()
{
  if (isTimeout()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2VisionServoActionNode] Timeout while waiting for result");

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
        "[R2VisionServoActionNode] Goal was rejected by action server");
      stage_ = Stage::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2VisionServoActionNode] Goal accepted, waiting for result...");

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
      const auto message = wrapped_result.result ? wrapped_result.result->message : "null result";
      const auto error_code = wrapped_result.result ? wrapped_result.result->error_code : 255;
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2VisionServoActionNode] Action did not succeed. result_code=%d error_code=%u message=%s",
        static_cast<int>(wrapped_result.code),
        error_code,
        message.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (!wrapped_result.result || !wrapped_result.result->success) {
      const auto message = wrapped_result.result ? wrapped_result.result->message : "null result";
      const auto error_code = wrapped_result.result ? wrapped_result.result->error_code : 255;
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2VisionServoActionNode] Vision servo failed. error_code=%u message=%s",
        error_code,
        message.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2VisionServoActionNode] Vision servo succeeded: %s final_error=(%.3f, %.3f, %.3f) yaw_error=%.3f",
      wrapped_result.result->message.c_str(),
      wrapped_result.result->final_error_x,
      wrapped_result.result->final_error_y,
      wrapped_result.result->final_error_z,
      wrapped_result.result->final_yaw_error_rad);

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void R2VisionServoActionNode::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[R2VisionServoActionNode] Halted, canceling goal if active");

  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }

  stage_ = Stage::IDLE;
}

bool R2VisionServoActionNode::isTimeout() const
{
  const auto now = std::chrono::steady_clock::now();
  return (now - start_time_) > result_timeout_;
}

}  // namespace r2_bt_nodes
