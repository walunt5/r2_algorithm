#ifndef R2_BT_NODES__CHASSIS_NODES_HPP_
#define R2_BT_NODES__CHASSIS_NODES_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "techx_r2_chassis_interfaces/action/step_command.hpp"

namespace r2_bt_nodes
{

class R2ChassisStepCommandNode : public BT::StatefulActionNode
{
public:
  using StepCommand = techx_r2_chassis_interfaces::action::StepCommand;
  using GoalHandleStepCommand = rclcpp_action::ClientGoalHandle<StepCommand>;

  R2ChassisStepCommandNode(
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
  rclcpp_action::Client<StepCommand>::SharedPtr client_;

  std::string action_name_;
  Stage stage_;

  std::shared_future<GoalHandleStepCommand::SharedPtr> goal_handle_future_;
  GoalHandleStepCommand::SharedPtr goal_handle_;
  std::shared_future<GoalHandleStepCommand::WrappedResult> result_future_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds result_timeout_;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__CHASSIS_NODES_HPP_