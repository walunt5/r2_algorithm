#ifndef R2_BT_NODES__NAV_NODES_HPP_
#define R2_BT_NODES__NAV_NODES_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "r2_nav_interfaces/action/navigate_to_pose.hpp"

namespace r2_bt_nodes
{

class R2NavigateToPoseActionNode : public BT::StatefulActionNode
{
public:
  using NavigateToPose = r2_nav_interfaces::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  R2NavigateToPoseActionNode(
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
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  std::string action_name_;
  Stage stage_;

  std::shared_future<GoalHandleNavigateToPose::SharedPtr> goal_handle_future_;
  GoalHandleNavigateToPose::SharedPtr goal_handle_;
  std::shared_future<GoalHandleNavigateToPose::WrappedResult> result_future_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds result_timeout_;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__NAV_NODES_HPP_