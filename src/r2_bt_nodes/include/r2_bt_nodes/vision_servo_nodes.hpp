#ifndef R2_BT_NODES__VISION_SERVO_NODES_HPP_
#define R2_BT_NODES__VISION_SERVO_NODES_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "r2_vision_servo_interfaces/action/vision_servo.hpp"

namespace r2_bt_nodes
{

class R2VisionServoActionNode : public BT::StatefulActionNode
{
public:
  using VisionServo = r2_vision_servo_interfaces::action::VisionServo;
  using GoalHandleVisionServo = rclcpp_action::ClientGoalHandle<VisionServo>;

  R2VisionServoActionNode(
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
  rclcpp_action::Client<VisionServo>::SharedPtr client_;

  std::string action_name_;
  Stage stage_;

  std::shared_future<GoalHandleVisionServo::SharedPtr> goal_handle_future_;
  GoalHandleVisionServo::SharedPtr goal_handle_;
  std::shared_future<GoalHandleVisionServo::WrappedResult> result_future_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds result_timeout_;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__VISION_SERVO_NODES_HPP_
