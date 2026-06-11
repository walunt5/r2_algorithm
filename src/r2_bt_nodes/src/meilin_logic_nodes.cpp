#include "r2_bt_nodes/meilin_logic_nodes.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>
#include <unordered_map>
#include <sstream>
#include <vector>

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

R2CheckBlockHasKfsNode::R2CheckBlockHasKfsNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2CheckBlockHasKfsNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("block", "Block name"),
    BT::OutputPort<bool>("has_kfs", "Whether this block has KFS")
  };
}

BT::NodeStatus R2CheckBlockHasKfsNode::tick()
{
  std::string block;

  if (!getInput("block", block)) {
    std::cerr << "[R2CheckBlockHasKfsNode] Missing input: block" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  // 第一版先硬编码。
  // 后面可以替换成读取 Meilin_12_Block_Map.yaml 或视觉输出。
  static const std::unordered_map<std::string, bool> block_kfs_map = {
    {"ENTRY", false},
    {"B1", false},
    {"B2", false},
    {"B3", true},
    {"B4", false},
    {"B5", true},
    {"B6", false},
    {"B7", false},
    {"B8", true},
    {"B9", false},
    {"B10", true},
    {"B11", false},
    {"B12", false},
    {"EXIT_ZONE", false}
  };

  const auto iter = block_kfs_map.find(block);

  if (iter == block_kfs_map.end()) {
    std::cerr
      << "[R2CheckBlockHasKfsNode] Unknown block="
      << block
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const bool has_kfs = iter->second;
  setOutput("has_kfs", has_kfs);

  std::cout
    << "[R2CheckBlockHasKfsNode] block="
    << block
    << " has_kfs="
    << (has_kfs ? "true" : "false")
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}


R2GetBlockKfsHeightNode::R2GetBlockKfsHeightNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2GetBlockKfsHeightNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("block", "Block name"),
    BT::OutputPort<int>("kfs_height", "KFS target height in mm")
  };
}

BT::NodeStatus R2GetBlockKfsHeightNode::tick()
{
  std::string block;

  if (!getInput("block", block)) {
    std::cerr << "[R2GetBlockKfsHeightNode] Missing input: block" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  // 第一版先硬编码。
  // 注意：这里只给“有 KFS 的方块”配置高度。
  static const std::unordered_map<std::string, int> block_kfs_height_map = {
    {"B3", 200},
    {"B5", 400},
    {"B8", 200},
    {"B10", 400}
  };

  const auto iter = block_kfs_height_map.find(block);

  if (iter == block_kfs_height_map.end()) {
    std::cerr
      << "[R2GetBlockKfsHeightNode] Block has no configured KFS height: "
      << block
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const int kfs_height = iter->second;
  setOutput("kfs_height", kfs_height);

  std::cout
    << "[R2GetBlockKfsHeightNode] block="
    << block
    << " kfs_height="
    << kfs_height
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}

static std::vector<std::string> splitRouteString(const std::string & route_text)
{
  std::vector<std::string> blocks;

  std::stringstream ss(route_text);
  std::string item;

  while (std::getline(ss, item, ',')) {
    const std::string trimmed = trimCopy(item);

    if (!trimmed.empty()) {
      blocks.push_back(trimmed);
    }
  }

  return blocks;
}


R2PeekFirstManualBlockNode::R2PeekFirstManualBlockNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2PeekFirstManualBlockNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "manual_block_sequence",
      "Manual block route string, for example: B2,B3,EXIT_ZONE"),
    BT::OutputPort<std::string>(
      "first_block",
      "First block in manual route")
  };
}

BT::NodeStatus R2PeekFirstManualBlockNode::tick()
{
  std::string manual_block_sequence;

  if (!getInput("manual_block_sequence", manual_block_sequence)) {
    std::cerr
      << "[R2PeekFirstManualBlockNode] Missing input: manual_block_sequence"
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const auto blocks = splitRouteString(manual_block_sequence);

  if (blocks.empty()) {
    std::cerr
      << "[R2PeekFirstManualBlockNode] Route is empty. manual_block_sequence="
      << manual_block_sequence
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const std::string first_block = blocks.front();

  setOutput("first_block", first_block);

  std::cout
    << "[R2PeekFirstManualBlockNode] manual_block_sequence="
    << manual_block_sequence
    << " first_block="
    << first_block
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}


R2GetNextManualBlockNode::R2GetNextManualBlockNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2GetNextManualBlockNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "manual_block_sequence",
      "Manual block route string, for example: B2,B3,EXIT_ZONE"),
    BT::InputPort<int>(
      "route_index",
      "Current route index, starting from 0"),
    BT::OutputPort<std::string>(
      "to_block",
      "Target block at route_index")
  };
}

BT::NodeStatus R2GetNextManualBlockNode::tick()
{
  std::string manual_block_sequence;
  int route_index = 0;

  if (!getInput("manual_block_sequence", manual_block_sequence)) {
    std::cerr
      << "[R2GetNextManualBlockNode] Missing input: manual_block_sequence"
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("route_index", route_index)) {
    std::cerr
      << "[R2GetNextManualBlockNode] Missing input: route_index"
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const auto blocks = splitRouteString(manual_block_sequence);

  if (blocks.empty()) {
    std::cerr
      << "[R2GetNextManualBlockNode] Route is empty. manual_block_sequence="
      << manual_block_sequence
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (route_index < 0) {
    std::cerr
      << "[R2GetNextManualBlockNode] Invalid route_index="
      << route_index
      << ". route_index must be >= 0."
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (static_cast<size_t>(route_index) >= blocks.size()) {
    std::cerr
      << "[R2GetNextManualBlockNode] route_index out of range. route_index="
      << route_index
      << " route_len="
      << blocks.size()
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const std::string to_block = blocks[route_index];

  setOutput("to_block", to_block);

  std::cout
    << "[R2GetNextManualBlockNode] manual_block_sequence="
    << manual_block_sequence
    << " route_index="
    << route_index
    << " to_block="
    << to_block
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}

struct TransitionInfo
{
  std::string edge_id;
  std::string approach_goal_name;
  std::string landing_goal_name;
};


R2GetTransitionInfoNode::R2GetTransitionInfoNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2GetTransitionInfoNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "from_block",
      "Current block name"),
    BT::InputPort<std::string>(
      "to_block",
      "Target block name"),
    BT::OutputPort<std::string>(
      "edge_id",
      "Transition edge id, for example: ENTRY_TO_B2"),
    BT::OutputPort<std::string>(
      "approach_goal_name",
      "Navigation goal name before climbing"),
    BT::OutputPort<std::string>(
      "landing_goal_name",
      "Navigation or alignment goal name after climbing")
  };
}

BT::NodeStatus R2GetTransitionInfoNode::tick()
{
  std::string from_block;
  std::string to_block;

  if (!getInput("from_block", from_block)) {
    std::cerr << "[R2GetTransitionInfoNode] Missing input: from_block" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("to_block", to_block)) {
    std::cerr << "[R2GetTransitionInfoNode] Missing input: to_block" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const std::string key = from_block + "->" + to_block;

  // 第一版先硬编码。后面再替换成读取 Meilin_12_Block_Map.yaml。
  static const std::unordered_map<std::string, TransitionInfo> transition_map = {
    {
      "ENTRY->B2",
      {
        "ENTRY_TO_B2",
        "ENTRY_TO_B2_APPROACH",
        "B2_LANDING"
      }
    },
    {
      "B2->B3",
      {
        "B2_TO_B3",
        "B2_TO_B3_APPROACH",
        "B3_LANDING"
      }
    },
    {
      "B3->EXIT_ZONE",
      {
        "B3_TO_EXIT_ZONE",
        "B3_TO_EXIT_ZONE_APPROACH",
        "EXIT_ZONE_LANDING"
      }
    }
  };

  const auto iter = transition_map.find(key);

  if (iter == transition_map.end()) {
    std::cerr
      << "[R2GetTransitionInfoNode] Unknown transition: "
      << key
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const auto & info = iter->second;

  setOutput("edge_id", info.edge_id);
  setOutput("approach_goal_name", info.approach_goal_name);
  setOutput("landing_goal_name", info.landing_goal_name);

  std::cout
    << "[R2GetTransitionInfoNode] from_block="
    << from_block
    << " to_block="
    << to_block
    << " edge_id="
    << info.edge_id
    << " approach_goal_name="
    << info.approach_goal_name
    << " landing_goal_name="
    << info.landing_goal_name
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace r2_bt_nodes