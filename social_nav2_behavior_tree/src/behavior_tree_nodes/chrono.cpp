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
#include "social_nav2_behavior_tree/behavior_tree_nodes/chrono.hpp"

namespace social_nav2_behavior_tree
{

Chrono::Chrono(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf),
  initialized_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), 
    "\"%s\" BtDecoratorNode initialized",
    name.c_str());
}
void Chrono::initialize() 
{
  pub_ = node_->create_publisher<std_msgs::msg::Float32>("/chrono/total_time", 1);
  initialized_ = true;
}

BT::NodeStatus Chrono::tick()
{
  if (!initialized_) {
    initialize();
  } else {
    std_msgs::msg::Float32 msg;
    msg.data = (node_->now() - first_tick_).seconds();
    pub_->publish(msg);
  }
  first_tick_ = node_->now();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace social_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<social_nav2_behavior_tree::Chrono>("Chrono");
}
