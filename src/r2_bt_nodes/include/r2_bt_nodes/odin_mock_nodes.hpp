#ifndef R2_BT_NODES__ODIN_MOCK_NODES_HPP_
#define R2_BT_NODES__ODIN_MOCK_NODES_HPP_

#include <string>

#include <behaviortree_cpp/bt_factory.h>

namespace r2_bt_nodes
{

class R2CheckOdinLocalizationOkMockNode : public BT::SyncActionNode
{
public:
  R2CheckOdinLocalizationOkMockNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};


class R2OdinPosePidAlignMockNode : public BT::SyncActionNode
{
public:
  R2OdinPosePidAlignMockNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace r2_bt_nodes

#endif  // R2_BT_NODES__ODIN_MOCK_NODES_HPP_