#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include "r2_bt_nodes/basic_nodes.hpp"
#include "r2_bt_nodes/arm_nodes.hpp"
#include "r2_bt_nodes/chassis_nodes.hpp"
#include "r2_bt_nodes/nav_nodes.hpp"
#include "r2_bt_nodes/meilin_logic_nodes.hpp"
#include "r2_bt_nodes/odin_mock_nodes.hpp"
#include "r2_bt_nodes/blackboard_nodes.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  // 1. 初始化 ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("r2_bt_executor_node");

  RCLCPP_INFO(node->get_logger(), "R2 behavior tree executor started.");

  // 2. 创建行为树工厂
  BT::BehaviorTreeFactory factory;

  // 3. 注册自定义节点
  factory.registerNodeType<r2_bt_nodes::R2ForceSuccess>("R2ForceSuccess");
  factory.registerNodeType<r2_bt_nodes::R2WaitForever>("R2WaitForever");

  factory.registerNodeType<r2_bt_nodes::R2CalculateHeightDeltaNode>(
    "R2CalculateHeightDeltaNode");

  factory.registerNodeType<r2_bt_nodes::R2BuildMeilinChassisCmdTypeNode>(
    "R2BuildMeilinChassisCmdTypeNode");

  factory.registerNodeType<r2_bt_nodes::R2BuildKfsPickActionIdByDeltaNode>(
    "R2BuildKfsPickActionIdByDeltaNode");

  factory.registerNodeType<r2_bt_nodes::R2BlackboardCheckStringNode>(
    "R2BlackboardCheckStringNode");

  factory.registerNodeType<r2_bt_nodes::R2BlackboardCheckBoolNode>(
    "R2BlackboardCheckBoolNode");

  factory.registerNodeType<r2_bt_nodes::R2CheckListNotEmptyNode>(
    "R2CheckListNotEmptyNode");

  factory.registerNodeType<r2_bt_nodes::R2GetBlockHeightNode>(
    "R2GetBlockHeightNode");

  factory.registerNodeType<r2_bt_nodes::R2GetBlockHeightFromYamlNode>(
    "R2GetBlockHeightFromYamlNode");

  factory.registerNodeType<r2_bt_nodes::R2CheckBlockHasKfsNode>(
    "R2CheckBlockHasKfsNode");

  factory.registerNodeType<r2_bt_nodes::R2GetBlockKfsHeightNode>(
    "R2GetBlockKfsHeightNode");

  factory.registerNodeType<r2_bt_nodes::R2CheckBlockHasKfsFromYamlNode>(
    "R2CheckBlockHasKfsFromYamlNode");

  factory.registerNodeType<r2_bt_nodes::R2GetBlockKfsHeightFromYamlNode>(
    "R2GetBlockKfsHeightFromYamlNode");

  factory.registerNodeType<r2_bt_nodes::R2PeekFirstManualBlockNode>(
    "R2PeekFirstManualBlockNode");
  
  factory.registerNodeType<r2_bt_nodes::R2GetNextManualBlockNode>(
    "R2GetNextManualBlockNode");

  factory.registerNodeType<r2_bt_nodes::R2CheckRouteIndexValidNode>(
    "R2CheckRouteIndexValidNode");

  factory.registerNodeType<r2_bt_nodes::R2CheckRouteFinishedNode>(
    "R2CheckRouteFinishedNode");

  factory.registerNodeType<r2_bt_nodes::R2GetTransitionInfoNode>(
    "R2GetTransitionInfoNode");

  factory.registerNodeType<r2_bt_nodes::R2CheckOdinLocalizationOkMockNode>(
    "R2CheckOdinLocalizationOkMockNode");
  
  factory.registerNodeType<r2_bt_nodes::R2OdinPosePidAlignMockNode>(
    "R2OdinPosePidAlignMockNode");

  factory.registerNodeType<r2_bt_nodes::R2SetBlackboardStringNode>(
    "R2SetBlackboardStringNode");
  
  factory.registerNodeType<r2_bt_nodes::R2SetBlackboardIntNode>(
    "R2SetBlackboardIntNode");
  
  factory.registerNodeType<r2_bt_nodes::R2IncrementIntNode>(
    "R2IncrementIntNode");

  factory.registerBuilder<r2_bt_nodes::R2GetHeadActionNode>(
    "R2GetHeadActionNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<r2_bt_nodes::R2GetHeadActionNode>(
        name,
        config,
        node);
    });

  factory.registerBuilder<r2_bt_nodes::R2SetEndEffectorNode>(
    "R2SetEndEffectorNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<r2_bt_nodes::R2SetEndEffectorNode>(
        name,
        config,
        node);
    });

  factory.registerBuilder<r2_bt_nodes::R2ExecuteArmActionNode>(
    "R2ExecuteArmActionNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<r2_bt_nodes::R2ExecuteArmActionNode>(
        name,
        config,
        node);
    });

  factory.registerBuilder<r2_bt_nodes::R2ChassisStepCommandNode>(
    "R2ChassisStepCommandNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<r2_bt_nodes::R2ChassisStepCommandNode>(
        name,
        config,
        node);
    });

  factory.registerBuilder<r2_bt_nodes::R2LiftControlNode>(
    "R2LiftControlNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<r2_bt_nodes::R2LiftControlNode>(
        name,
        config,
        node);
    });

  factory.registerBuilder<r2_bt_nodes::R2NavigateToPoseActionNode>(
    "R2NavigateToPoseActionNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<r2_bt_nodes::R2NavigateToPoseActionNode>(
        name,
        config,
        node);
    });

  // 4. 从 ROS 参数读取 XML 文件路径
  node->declare_parameter<std::string>("xml_file_path", "");
  std::string xml_file_path;
  node->get_parameter("xml_file_path", xml_file_path);

  if (xml_file_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "xml_file_path is empty. Please pass XML file path.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Loading behavior tree XML: %s", xml_file_path.c_str());

  // 5. 加载 XML 行为树
  auto tree = factory.createTreeFromFile(xml_file_path);

  // 6. 启动 Groot2 发布器
  BT::Groot2Publisher groot_publisher(tree,1667);

  // 7. 10Hz tick 行为树
  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    BT::NodeStatus status = tree.tickExactlyOnce();

    rclcpp::spin_some(node);

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Behavior tree finished with SUCCESS.");
      break;
    }

    if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "Behavior tree finished with FAILURE.");
      break;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}