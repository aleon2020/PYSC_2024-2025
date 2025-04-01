#include <string>
#include <iostream>

#include "trabajo_plansys2_envidio33/DoCube.hpp" 
#include "behaviortree_cpp/behavior_tree.h"

namespace trabajo_plansys2_envidio33
{

DoCube::DoCube(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
DoCube::halt()
{
  std::cout << "DoCube halt" << std::endl;
}

BT::NodeStatus
DoCube::tick()
{
  std::cout << "DoCube tick " << counter_ << std::endl;

  if (counter_++ < 5) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trabajo_plansys2_envidio33::DoCube>("DoCube");
}