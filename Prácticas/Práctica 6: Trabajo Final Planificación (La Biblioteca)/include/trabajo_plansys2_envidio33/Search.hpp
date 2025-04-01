#ifndef TRABAJO_PLANSYS2_ENVIDIO33__BEHAVIOR_TREE_NODES__SEARCH_HPP_
#define TRABAJO_PLANSYS2_ENVIDIO33__BEHAVIOR_TREE_NODES__SEARCH_HPP_

#include <string>
#include <random>
#include "behaviortree_cpp/behavior_tree.h"

namespace trabajo_plansys2_envidio33
{

class Search : public BT::ConditionNode
{
public:
  explicit Search(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("book") };
  }

  BT::NodeStatus tick() override;

private:
  std::mt19937 rng_;
  std::uniform_real_distribution<double> dist_;
};

}

#endif