#include "r2_bt_nodes/basic_nodes.hpp"

namespace r2_bt_nodes
{

R2ForceSuccess::R2ForceSuccess(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList R2ForceSuccess::providedPorts()
{
  return {};
}

BT::NodeStatus R2ForceSuccess::tick()
{
  return BT::NodeStatus::SUCCESS;
}


R2WaitForever::R2WaitForever(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList R2WaitForever::providedPorts()
{
  return {};
}

BT::NodeStatus R2WaitForever::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus R2WaitForever::onRunning()
{
  return BT::NodeStatus::RUNNING;
}

void R2WaitForever::onHalted()
{
}

}  // namespace r2_bt_nodes