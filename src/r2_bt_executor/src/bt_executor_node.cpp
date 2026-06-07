#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include "r2_bt_nodes/basic_nodes.hpp"

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