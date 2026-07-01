#ifndef R2_BT_NODES__TIMED_CMD_VEL_NODE_HPP_
#define R2_BT_NODES__TIMED_CMD_VEL_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace r2_bt_nodes
{

class R2TimedCmdVelNode : public BT::StatefulActionNode
{
public:
  R2TimedCmdVelNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  void publishCommand();

  void publishStop();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  std::string cmd_vel_topic_;

  double vx_;
  double vy_;
  double wz_;
  double duration_sec_;
  double rate_hz_;
  bool publish_stop_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_publish_time_;
  std::chrono::nanoseconds publish_period_;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__TIMED_CMD_VEL_NODE_HPP_