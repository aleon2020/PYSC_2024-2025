#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "trabajo_plansys2_envidio33/Move.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2_bt_tests
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: plansys2::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  if (!config().blackboard->get("node", node)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get 'node' from the blackboard");
  }

  try {
    node->declare_parameter<std::vector<std::string>>("waypoints");
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
  }

  if (node->has_parameter("waypoints")) {
    std::vector<std::string> wp_names;

    node->get_parameter_or("waypoints", wp_names, {});

    for (auto & wp : wp_names) {
      try {
        node->declare_parameter<std::vector<double>>("waypoint_coords." + wp);
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      }

      std::vector<double> coords;
      if (node->get_parameter_or("waypoint_coords." + wp, coords, {})) {
        geometry_msgs::msg::Pose2D pose;
        pose.x = coords[0];
        pose.y = coords[1];
        pose.theta = coords[2];

        waypoints_[wp] = pose;
      } else {
        std::cerr << "No coordinate configured for waypoint [" << wp << "]" << std::endl;
      }
    }
  }
}

BT::NodeStatus
Move::on_tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    if (!config().blackboard->get("node", node)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get 'node' from the blackboard");
    }

    std::string goal;
    getInput<std::string>("goal", goal);

    geometry_msgs::msg::Pose2D pose2nav;
    if (waypoints_.find(goal) != waypoints_.end()) {
      pose2nav = waypoints_[goal];
    } else {
      std::cerr << "No coordinate for waypoint [" << goal << "]" << std::endl;
    }

    geometry_msgs::msg::PoseStamped goal_pos;

    goal_pos.header.frame_id = "map";
    goal_pos.header.stamp = node->now();
    goal_pos.pose.position.x = pose2nav.x;
    goal_pos.pose.position.y = pose2nav.y;
    goal_pos.pose.position.z = 0;
    goal_pos.pose.orientation = tf2::toMsg(tf2::Quaternion({0.0, 0.0, 1.0}, pose2nav.theta));

    goal_.pose = goal_pos;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
Move::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<plansys2_bt_tests::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<plansys2_bt_tests::Move>(
    "Move", builder);
}