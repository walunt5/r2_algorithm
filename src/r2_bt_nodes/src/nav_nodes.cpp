#include "r2_bt_nodes/nav_nodes.hpp"

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>

namespace r2_bt_nodes
{

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

R2NavigateToPoseActionNode::R2NavigateToPoseActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  stage_(Stage::IDLE),
  result_timeout_(std::chrono::milliseconds(70000))
{
}

BT::PortsList R2NavigateToPoseActionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "action_name",
      "/r2_navigate_to_pose",
      "ROS2 action name for NavigateToPose"),
    BT::InputPort<std::string>(
      "goal_name",
      "",
      "Preset goal name. If empty, target_pose from frame_id/x/y/z/yaw will be used"),
    BT::InputPort<std::string>(
      "frame_id",
      "map",
      "Target pose frame id"),
    BT::InputPort<double>(
      "x",
      0.0,
      "Target pose x in meters"),
    BT::InputPort<double>(
      "y",
      0.0,
      "Target pose y in meters"),
    BT::InputPort<double>(
      "z",
      0.0,
      "Target pose z in meters"),
    BT::InputPort<double>(
      "yaw",
      0.0,
      "Target yaw in radians"),
    BT::InputPort<double>(
      "timeout_sec",
      60.0,
      "Navigation timeout in seconds"),
    BT::InputPort<int>(
      "server_timeout_ms",
      3000,
      "Timeout while waiting for action server"),
    BT::InputPort<int>(
      "result_timeout_ms",
      70000,
      "Timeout while waiting for action result")
  };
}

BT::NodeStatus R2NavigateToPoseActionNode::onStart()
{
  std::string action_name = "/r2_navigate_to_pose";
  std::string goal_name = "";
  std::string frame_id = "map";
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  double timeout_sec = 60.0;
  int server_timeout_ms = 3000;
  int result_timeout_ms = 70000;

  getInput("action_name", action_name);
  getInput("goal_name", goal_name);
  getInput("frame_id", frame_id);
  getInput("x", x);
  getInput("y", y);
  getInput("z", z);
  getInput("yaw", yaw);
  getInput("timeout_sec", timeout_sec);
  getInput("server_timeout_ms", server_timeout_ms);
  getInput("result_timeout_ms", result_timeout_ms);

  if (timeout_sec <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2NavigateToPoseActionNode] Invalid timeout_sec=%.2f. It must be > 0.",
      timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  if (server_timeout_ms <= 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2NavigateToPoseActionNode] Invalid server_timeout_ms=%d. It must be > 0.",
      server_timeout_ms);
    return BT::NodeStatus::FAILURE;
  }

  if (result_timeout_ms <= 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2NavigateToPoseActionNode] Invalid result_timeout_ms=%d. It must be > 0.",
      result_timeout_ms);
    return BT::NodeStatus::FAILURE;
  }

  result_timeout_ = std::chrono::milliseconds(result_timeout_ms);
  start_time_ = std::chrono::steady_clock::now();

  if (!client_ || action_name_ != action_name) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2NavigateToPoseActionNode] Waiting for action server: %s",
    action_name_.c_str());

  const auto server_timeout = std::chrono::milliseconds(server_timeout_ms);

  if (!client_->wait_for_action_server(server_timeout)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2NavigateToPoseActionNode] Action server not available: %s",
      action_name_.c_str());

    stage_ = Stage::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.goal_name = goal_name;
  goal_msg.timeout_sec = static_cast<float>(timeout_sec);

  goal_msg.target_pose.header.stamp = node_->get_clock()->now();
  goal_msg.target_pose.header.frame_id = frame_id;
  goal_msg.target_pose.pose.position.x = x;
  goal_msg.target_pose.pose.position.y = y;
  goal_msg.target_pose.pose.position.z = z;
  goal_msg.target_pose.pose.orientation = yawToQuaternion(yaw);

  if (!goal_name.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[R2NavigateToPoseActionNode] Sending goal by goal_name=%s timeout_sec=%.2f",
      goal_name.c_str(),
      timeout_sec);
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "[R2NavigateToPoseActionNode] Sending goal pose: frame=%s x=%.3f y=%.3f z=%.3f yaw=%.3f timeout_sec=%.2f",
      frame_id.c_str(),
      x,
      y,
      z,
      yaw,
      timeout_sec);
  }

  auto send_goal_options =
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.feedback_callback =
    [this](
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      if (!feedback) {
        return;
      }

      RCLCPP_INFO(
        node_->get_logger(),
        "[R2NavigateToPoseActionNode] feedback: state=%s distance=%.3f yaw_error=%.3f",
        feedback->state.c_str(),
        feedback->distance_to_goal,
        feedback->yaw_error);
    };

  goal_handle_future_ = client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();
  stage_ = Stage::WAIT_GOAL_ACCEPTED;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2NavigateToPoseActionNode::onRunning()
{
  if (isTimeout()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2NavigateToPoseActionNode] Timeout while waiting for NavigateToPose result.");

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
        "[R2NavigateToPoseActionNode] Goal was rejected by action server.");

      stage_ = Stage::IDLE;
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2NavigateToPoseActionNode] Goal accepted, waiting for result...");

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
        "[R2NavigateToPoseActionNode] Action did not succeed. result_code=%d",
        static_cast<int>(wrapped_result.code));

      return BT::NodeStatus::FAILURE;
    }

    const auto & result = wrapped_result.result;

    if (!result) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[R2NavigateToPoseActionNode] Result is null.");

      return BT::NodeStatus::FAILURE;
    }

    if (result->success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[R2NavigateToPoseActionNode] SUCCESS. message=%s",
        result->message.c_str());

      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2NavigateToPoseActionNode] FAILURE. message=%s",
      result->message.c_str());

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[R2NavigateToPoseActionNode] Invalid internal stage.");

  return BT::NodeStatus::FAILURE;
}

void R2NavigateToPoseActionNode::onHalted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[R2NavigateToPoseActionNode] Halted.");

  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }

  stage_ = Stage::IDLE;
}

bool R2NavigateToPoseActionNode::isTimeout() const
{
  const auto now = std::chrono::steady_clock::now();
  return (now - start_time_) > result_timeout_;
}

}  // namespace r2_bt_nodes