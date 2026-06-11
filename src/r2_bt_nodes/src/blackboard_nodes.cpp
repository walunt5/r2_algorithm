#include "r2_bt_nodes/blackboard_nodes.hpp"

#include <iostream>
#include <string>

namespace r2_bt_nodes
{

R2SetBlackboardStringNode::R2SetBlackboardStringNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2SetBlackboardStringNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "value",
      "Input string value"),
    BT::OutputPort<std::string>(
      "output",
      "Output string value")
  };
}

BT::NodeStatus R2SetBlackboardStringNode::tick()
{
  std::string value;

  if (!getInput("value", value)) {
    std::cerr
      << "[R2SetBlackboardStringNode] Missing input: value"
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  setOutput("output", value);

  std::cout
    << "[R2SetBlackboardStringNode] output="
    << value
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}


R2SetBlackboardIntNode::R2SetBlackboardIntNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2SetBlackboardIntNode::providedPorts()
{
  return {
    BT::InputPort<int>(
      "value",
      "Input int value"),
    BT::OutputPort<int>(
      "output",
      "Output int value")
  };
}

BT::NodeStatus R2SetBlackboardIntNode::tick()
{
  int value = 0;

  if (!getInput("value", value)) {
    std::cerr
      << "[R2SetBlackboardIntNode] Missing input: value"
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  setOutput("output", value);

  std::cout
    << "[R2SetBlackboardIntNode] output="
    << value
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}


R2IncrementIntNode::R2IncrementIntNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2IncrementIntNode::providedPorts()
{
  return {
    BT::InputPort<int>(
      "input",
      "Input int value"),
    BT::InputPort<int>(
      "step",
      1,
      "Increment step"),
    BT::OutputPort<int>(
      "output",
      "Output int value")
  };
}

BT::NodeStatus R2IncrementIntNode::tick()
{
  int input = 0;
  int step = 1;

  if (!getInput("input", input)) {
    std::cerr
      << "[R2IncrementIntNode] Missing input: input"
      << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  getInput("step", step);

  const int output = input + step;
  setOutput("output", output);

  std::cout
    << "[R2IncrementIntNode] input="
    << input
    << " step="
    << step
    << " output="
    << output
    << std::endl;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace r2_bt_nodes