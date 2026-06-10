#ifndef R2_BT_NODES__MEILIN_LOGIC_NODES_HPP_
#define R2_BT_NODES__MEILIN_LOGIC_NODES_HPP_

#include <string>

#include <behaviortree_cpp/bt_factory.h>

namespace r2_bt_nodes
{

class R2CalculateHeightDeltaNode : public BT::SyncActionNode
{
public:
  R2CalculateHeightDeltaNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2BuildMeilinChassisCmdTypeNode : public BT::SyncActionNode
{
public:
  R2BuildMeilinChassisCmdTypeNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2BuildKfsPickActionIdByDeltaNode : public BT::SyncActionNode
{
public:
  R2BuildKfsPickActionIdByDeltaNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class R2BlackboardCheckStringNode : public BT::SyncActionNode
{
public:
  R2BlackboardCheckStringNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2BlackboardCheckBoolNode : public BT::SyncActionNode
{
public:
  R2BlackboardCheckBoolNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2CheckListNotEmptyNode : public BT::SyncActionNode
{
public:
  R2CheckListNotEmptyNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2GetBlockHeightNode : public BT::SyncActionNode
{
public:
  R2GetBlockHeightNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class R2CheckBlockHasKfsNode : public BT::SyncActionNode
{
public:
  R2CheckBlockHasKfsNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2GetBlockKfsHeightNode : public BT::SyncActionNode
{
public:
  R2GetBlockKfsHeightNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__MEILIN_LOGIC_NODES_HPP_