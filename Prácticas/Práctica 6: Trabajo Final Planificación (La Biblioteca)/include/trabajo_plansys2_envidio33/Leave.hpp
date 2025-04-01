#ifndef TRABAJO_PLANSYS2_ENVIDIO33__BEHAVIOR_TREE_NODES__LEAVE_HPP_
#define TRABAJO_PLANSYS2_ENVIDIO33__BEHAVIOR_TREE_NODES__LEAVE_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace trabajo_plansys2_envidio33
{

class Leave : public BT::ActionNodeBase
{
public:
  explicit Leave(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  int counter_;
};

}

#endif