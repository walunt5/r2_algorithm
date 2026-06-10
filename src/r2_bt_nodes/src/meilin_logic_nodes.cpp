#include "r2_bt_nodes/meilin_logic_nodes.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>
#include <unordered_map>

namespace r2_bt_nodes
{

R2CalculateHeightDeltaNode::R2CalculateHeightDeltaNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2CalculateHeightDeltaNode::providedPorts()
{
  return {
    BT::InputPort<int>("current_h", "Current block height in mm"),
    BT::InputPort<int>("next_h", "Next block or target height in mm"),
    BT::OutputPort<int>("delta_h", "Height delta in mm, next_h - current_h")
  };
}

BT::NodeStatus R2CalculateHeightDeltaNode::tick()
{
  int current_h = 0;
  int next_h = 0;

  if (!getInput("current_h", current_h)) {
    std::cerr << "[R2CalculateHeightDeltaNode] Missing input: current_h" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("next_h", next_h)) {
    std::cerr << "[R2CalculateHeightDeltaNode] Missing input: next_h" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const int delta_h = next_h - current_h;
  setOutput("delta_h", delta_h);

  std::cout
    << "[R2CalculateHeightDeltaNode] current_h=" << current_h
    << " next_h=" << next_h
    << " delta_h=" << delta_h
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}


R2BuildMeilinChassisCmdTypeNode::R2BuildMeilinChassisCmdTypeNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2BuildMeilinChassisCmdTypeNode::providedPorts()
{
  return {
    BT::InputPort<int>("delta_h", "Height delta in mm"),
    BT::OutputPort<std::string>("cmd_type", "Chassis step command type")
  };
}

BT::NodeStatus R2BuildMeilinChassisCmdTypeNode::tick()
{
  int delta_h = 0;

  if (!getInput("delta_h", delta_h)) {
    std::cerr << "[R2BuildMeilinChassisCmdTypeNode] Missing input: delta_h" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::string cmd_type;

  if (delta_h == 0) {
    cmd_type = "MOVE_FLAT";
  } else if (delta_h == 200) {
    cmd_type = "CLIMB_200";
  } else if (delta_h == 400) {
    cmd_type = "CLIMB_400";
  } else if (delta_h == -200) {
    cmd_type = "DESCEND_200";
  } else if (delta_h == -400) {
    cmd_type = "DESCEND_400";
  } else {
    std::cerr
      << "[R2BuildMeilinChassisCmdTypeNode] Invalid delta_h="
      << delta_h
      << ". Valid values: 0, 200, 400, -200, -400."
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  setOutput("cmd_type", cmd_type);

  std::cout
    << "[R2BuildMeilinChassisCmdTypeNode] delta_h=" << delta_h
    << " cmd_type=" << cmd_type
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}


R2BuildKfsPickActionIdByDeltaNode::R2BuildKfsPickActionIdByDeltaNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2BuildKfsPickActionIdByDeltaNode::providedPorts()
{
  return {
    BT::InputPort<int>("delta_h", "KFS height delta in mm"),
    BT::OutputPort<int>("action_id", "Arm action id for KFS pick")
  };
}

BT::NodeStatus R2BuildKfsPickActionIdByDeltaNode::tick()
{
  int delta_h = 0;

  if (!getInput("delta_h", delta_h)) {
    std::cerr << "[R2BuildKfsPickActionIdByDeltaNode] Missing input: delta_h" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  int action_id = 0;

  if (delta_h == 0) {
    action_id = 0x0210;       // 528
  } else if (delta_h == 200) {
    action_id = 0x0211;       // 529
  } else if (delta_h == -200) {
    action_id = 0x0212;       // 530
  } else if (delta_h == 400) {
    action_id = 0x0213;       // 531
  } else if (delta_h == -400) {
    action_id = 0x0214;       // 532
  } else {
    std::cerr
      << "[R2BuildKfsPickActionIdByDeltaNode] Invalid delta_h="
      << delta_h
      << ". Valid values: 0, 200, -200, 400, -400."
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  setOutput("action_id", action_id);

  std::cout
    << "[R2BuildKfsPickActionIdByDeltaNode] delta_h=" << delta_h
    << " action_id=" << action_id
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}

R2BlackboardCheckStringNode::R2BlackboardCheckStringNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2BlackboardCheckStringNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("value", "String value from blackboard"),
    BT::InputPort<std::string>("expected", "Expected string value")
  };
}

BT::NodeStatus R2BlackboardCheckStringNode::tick()
{
  std::string value;
  std::string expected;

  if (!getInput("value", value)) {
    std::cerr << "[R2BlackboardCheckStringNode] Missing input: value" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("expected", expected)) {
    std::cerr << "[R2BlackboardCheckStringNode] Missing input: expected" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (value == expected) {
    std::cout
      << "[R2BlackboardCheckStringNode] SUCCESS. value="
      << value
      << " expected="
      << expected
      << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  std::cout
    << "[R2BlackboardCheckStringNode] FAILURE. value="
    << value
    << " expected="
    << expected
    << std::endl;

  return BT::NodeStatus::FAILURE;
}


R2BlackboardCheckBoolNode::R2BlackboardCheckBoolNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2BlackboardCheckBoolNode::providedPorts()
{
  return {
    BT::InputPort<bool>("value", "Bool value from blackboard"),
    BT::InputPort<bool>("expected", "Expected bool value")
  };
}

BT::NodeStatus R2BlackboardCheckBoolNode::tick()
{
  bool value = false;
  bool expected = false;

  if (!getInput("value", value)) {
    std::cerr << "[R2BlackboardCheckBoolNode] Missing input: value" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("expected", expected)) {
    std::cerr << "[R2BlackboardCheckBoolNode] Missing input: expected" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (value == expected) {
    std::cout
      << "[R2BlackboardCheckBoolNode] SUCCESS. value="
      << (value ? "true" : "false")
      << " expected="
      << (expected ? "true" : "false")
      << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  std::cout
    << "[R2BlackboardCheckBoolNode] FAILURE. value="
    << (value ? "true" : "false")
    << " expected="
    << (expected ? "true" : "false")
    << std::endl;

  return BT::NodeStatus::FAILURE;
}


R2CheckListNotEmptyNode::R2CheckListNotEmptyNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2CheckListNotEmptyNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("text", "Route string, for example: B2,B6,EXIT_ZONE")
  };
}

static std::string trimCopy(const std::string & input)
{
  std::string output = input;

  output.erase(
    output.begin(),
    std::find_if(
      output.begin(),
      output.end(),
      [](unsigned char ch) {
        return !std::isspace(ch);
      }));

  output.erase(
    std::find_if(
      output.rbegin(),
      output.rend(),
      [](unsigned char ch) {
        return !std::isspace(ch);
      }).base(),
    output.end());

  return output;
}

BT::NodeStatus R2CheckListNotEmptyNode::tick()
{
  std::string text;

  if (!getInput("text", text)) {
    std::cerr << "[R2CheckListNotEmptyNode] Missing input: text" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const std::string trimmed = trimCopy(text);

  if (trimmed.empty()) {
    std::cout << "[R2CheckListNotEmptyNode] FAILURE. list is empty." << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::cout
    << "[R2CheckListNotEmptyNode] SUCCESS. text="
    << trimmed
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}


R2GetBlockHeightNode::R2GetBlockHeightNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2GetBlockHeightNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("block", "Block name"),
    BT::OutputPort<int>("height", "Block height in mm")
  };
}

BT::NodeStatus R2GetBlockHeightNode::tick()
{
  std::string block;

  if (!getInput("block", block)) {
    std::cerr << "[R2GetBlockHeightNode] Missing input: block" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  // 第一版先硬编码。后面再替换成读取 Meilin_12_Block_Map.yaml。
  static const std::unordered_map<std::string, int> block_height_map = {
    {"ENTRY", 0},
    {"B1", 0},
    {"B2", 200},
    {"B3", 200},
    {"B4", 400},
    {"B5", 400},
    {"B6", 200},
    {"B7", 0},
    {"B8", 200},
    {"B9", 400},
    {"B10", 200},
    {"B11", 0},
    {"B12", 0},
    {"EXIT_ZONE", 0}
  };

  const auto iter = block_height_map.find(block);

  if (iter == block_height_map.end()) {
    std::cerr
      << "[R2GetBlockHeightNode] Unknown block="
      << block
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const int height = iter->second;
  setOutput("height", height);

  std::cout
    << "[R2GetBlockHeightNode] block="
    << block
    << " height="
    << height
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace r2_bt_nodes