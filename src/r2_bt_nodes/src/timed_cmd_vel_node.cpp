#include "r2_bt_nodes/timed_cmd_vel_node.hpp"

namespace r2_bt_nodes
{

R2TimedCmdVelNode::R2TimedCmdVelNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),
  node_(node),
  cmd_vel_topic_("/cmd_vel"),
  vx_(0.0),
  vy_(0.0),
  wz_(0.0),
  duration_sec_(0.0),
  rate_hz_(20.0),
  publish_stop_(true),
  publish_period_(std::chrono::milliseconds(50))
{
}

BT::PortsList R2TimedCmdVelNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "cmd_vel_topic",
      "/cmd_vel",
      "cmd_vel topic name"),
    BT::InputPort<double>(
      "vx",
      0.0,
      "linear x velocity, m/s. Positive means forward"),
    BT::InputPort<double>(
      "vy",
      0.0,
      "linear y velocity, m/s. Positive means left"),
    BT::InputPort<double>(
      "wz",
      0.0,
      "angular z velocity, rad/s. Positive means counter-clockwise"),
    BT::InputPort<double>(
      "duration_sec",
      0.0,
      "duration in seconds"),
    BT::InputPort<double>(
      "rate_hz",
      20.0,
      "publish rate in Hz"),
    BT::InputPort<bool>(
      "publish_stop",
      true,
      "publish zero Twist when finished or halted")
  };
}

BT::NodeStatus R2TimedCmdVelNode::onStart()
{
  std::string cmd_vel_topic = "/cmd_vel";
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  double duration_sec = 0.0;
  double rate_hz = 20.0;
  bool publish_stop = true;

  getInput("cmd_vel_topic", cmd_vel_topic);
  getInput("vx", vx);
  getInput("vy", vy);
  getInput("wz", wz);
  getInput("duration_sec", duration_sec);
  getInput("rate_hz", rate_hz);
  getInput("publish_stop", publish_stop);

  if (cmd_vel_topic.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2TimedCmdVelNode] cmd_vel_topic is empty.");
    return BT::NodeStatus::FAILURE;
  }

  if (duration_sec <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2TimedCmdVelNode] Invalid duration_sec=%.3f. It must be > 0.",
      duration_sec);
    return BT::NodeStatus::FAILURE;
  }

  if (rate_hz <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[R2TimedCmdVelNode] Invalid rate_hz=%.3f. It must be > 0.",
      rate_hz);
    return BT::NodeStatus::FAILURE;
  }

  cmd_vel_topic_ = cmd_vel_topic;
  vx_ = vx;
  vy_ = vy;
  wz_ = wz;
  duration_sec_ = duration_sec;
  rate_hz_ = rate_hz;
  publish_stop_ = publish_stop;

  publish_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz_));

  if (!publisher_) {
    publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::QoS(10));
  }

  start_time_ = std::chrono::steady_clock::now();
  last_publish_time_ = start_time_ - publish_period_;

  RCLCPP_INFO(
    node_->get_logger(),
    "[R2TimedCmdVelNode] Start timed cmd_vel: topic=%s vx=%.3f vy=%.3f wz=%.3f "
    "duration=%.3f sec rate=%.3f Hz",
    cmd_vel_topic_.c_str(),
    vx_,
    vy_,
    wz_,
    duration_sec_,
    rate_hz_);

  publishCommand();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2TimedCmdVelNode::onRunning()
{
  const auto now = std::chrono::steady_clock::now();
  const double elapsed_sec =
    std::chrono::duration<double>(now - start_time_).count();

  if (elapsed_sec >= duration_sec_) {
    if (publish_stop_) {
      publishStop();
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[R2TimedCmdVelNode] Finished timed cmd_vel. elapsed=%.3f sec",
      elapsed_sec);

    return BT::NodeStatus::SUCCESS;
  }

  if ((now - last_publish_time_) >= publish_period_) {
    publishCommand();
  }

  return BT::NodeStatus::RUNNING;
}

void R2TimedCmdVelNode::onHalted()
{
  if (publish_stop_) {
    publishStop();
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "[R2TimedCmdVelNode] Halted. Published zero Twist.");
}

void R2TimedCmdVelNode::publishCommand()
{
  if (!publisher_) {
    return;
  }

  geometry_msgs::msg::Twist msg;
  msg.linear.x = vx_;
  msg.linear.y = vy_;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = wz_;

  publisher_->publish(msg);
  last_publish_time_ = std::chrono::steady_clock::now();
}

void R2TimedCmdVelNode::publishStop()
{
  if (!publisher_) {
    return;
  }

  geometry_msgs::msg::Twist msg;
  publisher_->publish(msg);
}

}  // namespace r2_bt_nodes