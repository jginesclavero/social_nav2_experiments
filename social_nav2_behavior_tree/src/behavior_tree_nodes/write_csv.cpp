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
#include "social_nav2_behavior_tree/behavior_tree_nodes/write_csv.hpp"

namespace social_nav2_behavior_tree
{

WriteCSV::WriteCSV(
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
void WriteCSV::initialize() 
{
  pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    "/social_nav_exp/task_finished",
    rclcpp::SystemDefaultsQoS());
  initialized_ = true;
}

BT::NodeStatus WriteCSV::tick()
{
  if (!initialized_) {
    initialize();
  }
  setStatus(BT::NodeStatus::SUCCESS);
  std_msgs::msg::Empty msg;
  pub_->publish(msg);

  return status();
}

}  // namespace social_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<social_nav2_behavior_tree::WriteCSV>("WriteCSV");
}
