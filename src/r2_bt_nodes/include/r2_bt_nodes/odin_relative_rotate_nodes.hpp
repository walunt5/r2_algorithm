#ifndef R2_BT_NODES__ODIN_RELATIVE_ROTATE_NODES_HPP_
#define R2_BT_NODES__ODIN_RELATIVE_ROTATE_NODES_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "r2_odin_interfaces/action/odin_relative_rotate.hpp"

namespace r2_bt_nodes
{

class R2OdinRelativeRotateActionNode : public BT::StatefulActionNode
{
public:
  using OdinRelativeRotate =
    r2_odin_interfaces::action::OdinRelativeRotate;

  using GoalHandle =
    rclcpp_action::ClientGoalHandle<OdinRelativeRotate>;

  R2OdinRelativeRotateActionNode(
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

  void cancelActiveGoal();

  void resetOutputs();

  void setFailureOutputs(
    const std::string & message);

  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<
    OdinRelativeRotate>::SharedPtr client_;

  std::string action_name_;

  Stage stage_{Stage::IDLE};

  std::shared_future<
    GoalHandle::SharedPtr> goal_handle_future_;

  GoalHandle::SharedPtr goal_handle_;

  std::shared_future<
    GoalHandle::WrappedResult> result_future_;

  std::chrono::steady_clock::time_point start_time_;

  std::chrono::milliseconds result_timeout_{
    std::chrono::milliseconds(10000)};
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__ODIN_RELATIVE_ROTATE_NODES_HPP_
