#include "r2_bt_nodes/odin_mock_nodes.hpp"

#include <iostream>
#include <string>

namespace r2_bt_nodes
{

R2CheckOdinLocalizationOkMockNode::R2CheckOdinLocalizationOkMockNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2CheckOdinLocalizationOkMockNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "source_frame",
      "Source frame, usually map"),
    BT::InputPort<std::string>(
      "target_frame",
      "Target frame, usually odin"),
    BT::InputPort<int>(
      "timeout_ms",
      1000,
      "Mock timeout in milliseconds")
  };
}

BT::NodeStatus R2CheckOdinLocalizationOkMockNode::tick()
{
  std::string source_frame = "map";
  std::string target_frame = "odin";
  int timeout_ms = 1000;

  getInput("source_frame", source_frame);
  getInput("target_frame", target_frame);
  getInput("timeout_ms", timeout_ms);

  std::cout
    << "[R2CheckOdinLocalizationOkMockNode] Mock localization OK. "
    << "source_frame="
    << source_frame
    << " target_frame="
    << target_frame
    << " timeout_ms="
    << timeout_ms
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}


R2OdinPosePidAlignMockNode::R2OdinPosePidAlignMockNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2OdinPosePidAlignMockNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "edge_id",
      "Transition edge id, only used for log in mock version"),
    BT::InputPort<std::string>(
      "goal_name",
      "Target goal name, for example ENTRY_TO_B2_APPROACH"),
    BT::InputPort<int>(
      "timeout_ms",
      3000,
      "Mock align timeout in milliseconds")
  };
}

BT::NodeStatus R2OdinPosePidAlignMockNode::tick()
{
  std::string edge_id;
  std::string goal_name;
  int timeout_ms = 3000;

  if (!getInput("goal_name", goal_name)) {
    std::cerr
      << "[R2OdinPosePidAlignMockNode] Missing input: goal_name"
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  getInput("edge_id", edge_id);
  getInput("timeout_ms", timeout_ms);

  std::cout
    << "[R2OdinPosePidAlignMockNode] Mock align SUCCESS. "
    << "edge_id="
    << edge_id
    << " goal_name="
    << goal_name
    << " timeout_ms="
    << timeout_ms
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace r2_bt_nodes