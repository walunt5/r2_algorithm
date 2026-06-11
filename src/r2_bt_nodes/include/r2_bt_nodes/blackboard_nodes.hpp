#ifndef R2_BT_NODES__BLACKBOARD_NODES_HPP_
#define R2_BT_NODES__BLACKBOARD_NODES_HPP_

#include <string>

#include <behaviortree_cpp/bt_factory.h>

namespace r2_bt_nodes
{

class R2SetBlackboardStringNode : public BT::SyncActionNode
{
public:
  R2SetBlackboardStringNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2SetBlackboardIntNode : public BT::SyncActionNode
{
public:
  R2SetBlackboardIntNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2IncrementIntNode : public BT::SyncActionNode
{
public:
  R2IncrementIntNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__BLACKBOARD_NODES_HPP_