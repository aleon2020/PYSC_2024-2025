#include <string>
#include <iostream>
#include <memory>
#include <random>

#include "behaviortree_cpp/behavior_tree.h"

namespace trabajo_plansys2_envidio33
{

class Search : public BT::ConditionNode
{
public:
  Search(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(xml_tag_name, conf), rng_(std::random_device{}()), dist_(0.0, 1.0)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("book") };
  }

  BT::NodeStatus tick() override
  {
    std::string book;
    getInput("book", book);

    if (dist_(rng_) <= 0.15) {
      std::cout << book << " found " << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      std::cout << "Search failed " << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  std::mt19937 rng_;
  std::uniform_real_distribution<double> dist_;
};

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trabajo_plansys2_envidio33::Search>("Search");
}
