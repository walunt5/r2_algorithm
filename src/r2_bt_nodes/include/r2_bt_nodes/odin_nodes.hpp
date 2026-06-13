#ifndef R2_BT_NODES__ODIN_NODES_HPP_
#define R2_BT_NODES__ODIN_NODES_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "r2_odin_interfaces/action/odin_pose_pid_align.hpp"

namespace r2_bt_nodes
{

class R2OdinPosePidAlignActionNode : public BT::StatefulActionNode
{
public:
  using OdinPosePidAlign = r2_odin_interfaces::action::OdinPosePidAlign;
  using GoalHandleAlign = rclcpp_action::ClientGoalHandle<OdinPosePidAlign>;

  R2OdinPosePidAlignActionNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Stage
  {
    IDLE,
    WAIT_GOAL_ACCEPTED,
    WAIT_RESULT
  };

  bool isTimeout() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<OdinPosePidAlign>::SharedPtr client_;

  std::string action_name_;
  Stage stage_;

  std::shared_future<GoalHandleAlign::SharedPtr> goal_handle_future_;
  GoalHandleAlign::SharedPtr goal_handle_;
  std::shared_future<GoalHandleAlign::WrappedResult> result_future_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds result_timeout_;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__ODIN_NODES_HPP_