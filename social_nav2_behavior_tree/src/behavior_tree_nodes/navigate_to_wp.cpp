// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "social_nav2_behavior_tree/behavior_tree_nodes/navigate_to_wp.hpp"

namespace social_nav2_behavior_tree
{

NavigateToWp::NavigateToWp(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  
}

void
NavigateToWp::on_tick()
{
  auto wp_map = 
    config().blackboard->get<std::unordered_map<std::string, geometry_msgs::msg::Pose>>("wp_map");
  
  auto res = getInput<std::string>("goal").value();
  wp_ = wp_map[res];
  RCLCPP_INFO(node_->get_logger(), "Navigating to... [%s -- %f %f]",
   res.c_str(), wp_.position.x, wp_.position.y);
  goal_.pose.pose = wp_;
  goal_.behavior_tree = "/home/jgines/social_nav2/src/social_nav2/social_nav2_bringup/behavior_trees/navigate_w_replanning_and_recovery.xml";
}

BT::NodeStatus
NavigateToWp::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace social_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<social_nav2_behavior_tree::NavigateToWp>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<social_nav2_behavior_tree::NavigateToWp>(
    "NavigateToWp", builder);
}
