#ifndef R2_BT_NODES__ARM_NODES_HPP_
#define R2_BT_NODES__ARM_NODES_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "techx_r2_arm_interfaces/action/get_head.hpp"
#include "techx_r2_arm_interfaces/action/set_end_effector.hpp"
#include "techx_r2_arm_interfaces/action/execute_action.hpp"

namespace r2_bt_nodes
{

class R2GetHeadActionNode : public BT::StatefulActionNode
{
public:
  using GetHead = techx_r2_arm_interfaces::action::GetHead;
  using GoalHandleGetHead = rclcpp_action::ClientGoalHandle<GetHead>;

  R2GetHeadActionNode(
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
  rclcpp_action::Client<GetHead>::SharedPtr client_;

  std::string action_name_;
  Stage stage_;

  std::shared_future<GoalHandleGetHead::SharedPtr> goal_handle_future_;
  GoalHandleGetHead::SharedPtr goal_handle_;
  std::shared_future<GoalHandleGetHead::WrappedResult> result_future_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds result_timeout_;
};

class R2SetEndEffectorNode : public BT::StatefulActionNode
{
public:
  using SetEndEffector = techx_r2_arm_interfaces::action::SetEndEffector;
  using GoalHandleSetEndEffector = rclcpp_action::ClientGoalHandle<SetEndEffector>;

  R2SetEndEffectorNode(
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
  rclcpp_action::Client<SetEndEffector>::SharedPtr client_;

  std::string action_name_;
  Stage stage_;

  std::shared_future<GoalHandleSetEndEffector::SharedPtr> goal_handle_future_;
  GoalHandleSetEndEffector::SharedPtr goal_handle_;
  std::shared_future<GoalHandleSetEndEffector::WrappedResult> result_future_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds result_timeout_;
};

class R2ExecuteArmActionNode : public BT::StatefulActionNode
{
public:
  using ExecuteAction = techx_r2_arm_interfaces::action::ExecuteAction;
  using GoalHandleExecuteAction = rclcpp_action::ClientGoalHandle<ExecuteAction>;

  R2ExecuteArmActionNode(
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
  rclcpp_action::Client<ExecuteAction>::SharedPtr client_;

  std::string action_name_;
  Stage stage_;

  std::shared_future<GoalHandleExecuteAction::SharedPtr> goal_handle_future_;
  GoalHandleExecuteAction::SharedPtr goal_handle_;
  std::shared_future<GoalHandleExecuteAction::WrappedResult> result_future_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds result_timeout_;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__ARM_NODES_HPP_