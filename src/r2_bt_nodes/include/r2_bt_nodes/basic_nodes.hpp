#ifndef R2_BT_NODES__BASIC_NODES_HPP_
#define R2_BT_NODES__BASIC_NODES_HPP_

#include <string>

#include <behaviortree_cpp/bt_factory.h>

namespace r2_bt_nodes
{

class R2ForceSuccess : public BT::SyncActionNode
{
public:
  R2ForceSuccess(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2WaitForever : public BT::StatefulActionNode
{
public:
  R2WaitForever(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__BASIC_NODES_HPP_