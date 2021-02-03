// Copyright (c) 2019 Intel Corporation
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
#include "social_nav2_behavior_tree/behavior_tree_nodes/simple_goal_reached_condition.hpp"

namespace social_nav2_behavior_tree
{

SimpleGoalReached::SimpleGoalReached(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  goal_reached_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), 
    "\"%s\" BtConditionNode initialized",
    condition_name.c_str());
  initialize();
}

void SimpleGoalReached::callback(const rcl_interfaces::msg::Log::SharedPtr msg)
{
  goal_reached_ = (msg->name == "bt_navigator" && msg->msg == "Navigation succeeded");
}

BT::NodeStatus SimpleGoalReached::tick()
{
  if (!initialized_)
    initialize();
  if (isGoalReached()) {
    goal_reached_ = false;
    initialized_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void SimpleGoalReached::initialize()
{
  sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::SystemDefaultsQoS(),
    std::bind(&SimpleGoalReached::callback, this, std::placeholders::_1));
  goal_reached_ = false;
  initialized_ = true;
}

}  // namespace social_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<social_nav2_behavior_tree::SimpleGoalReached>("isGoalReached");
}
