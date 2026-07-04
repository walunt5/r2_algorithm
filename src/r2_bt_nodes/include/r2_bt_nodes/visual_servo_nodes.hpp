#ifndef R2_BT_NODES__VISUAL_SERVO_NODES_HPP_
#define R2_BT_NODES__VISUAL_SERVO_NODES_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "gmk_visual_servo_interfaces/action/visual_servo.hpp"

namespace r2_bt_nodes
{

class R2WeaponVisualServoActionNode : public BT::StatefulActionNode
{
public:
  using VisualServo = gmk_visual_servo_interfaces::action::VisualServo;
  using GoalHandle = rclcpp_action::ClientGoalHandle<VisualServo>;

  R2WeaponVisualServoActionNode(
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

  void setClientFailureOutputs(const std::string & message);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<VisualServo>::SharedPtr client_;

  std::string action_name_;
  Stage stage_;

  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds result_timeout_;

  // 每次开始请求时递增。
  // Halt、超时或结束时再次递增，使迟到回调失效。
  std::atomic<std::uint64_t> request_generation_;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__VISUAL_SERVO_NODES_HPP_