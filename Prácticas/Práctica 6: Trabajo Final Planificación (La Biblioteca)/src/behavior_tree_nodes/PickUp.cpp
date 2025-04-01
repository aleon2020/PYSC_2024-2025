#include <string>
#include <iostream>

#include "trabajo_plansys2_envidio33/PickUp.hpp" 
#include "behaviortree_cpp/behavior_tree.h"

namespace trabajo_plansys2_envidio33
{

PickUp::PickUp(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
PickUp::halt()
{
  std::cout << "PickUp halt" << std::endl;
}

BT::NodeStatus
PickUp::tick()
{
  std::cout << "PickUp tick " << counter_ << std::endl;

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
  factory.registerNodeType<trabajo_plansys2_envidio33::PickUp>("PickUp");
}