#ifndef R2_BT_NODES__LIGHT_SIGNAL_NODES_HPP_
#define R2_BT_NODES__LIGHT_SIGNAL_NODES_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "r2_light_interfaces/action/wait_for_light_signal.hpp"

namespace r2_bt_nodes
{

class R2WaitForLightSignalActionNode : public BT::StatefulActionNode
{
public:
  using WaitForLightSignal = r2_light_interfaces::action::WaitForLightSignal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<WaitForLightSignal>;

  R2WaitForLightSignalActionNode(
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
  rclcpp_action::Client<WaitForLightSignal>::SharedPtr client_;
  std::string action_name_;
  Stage stage_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds result_timeout_;
  std::atomic<std::uint64_t> request_generation_;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__LIGHT_SIGNAL_NODES_HPP_
