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
#include "social_nav2_behavior_tree/behavior_tree_nodes/stop_condition.hpp"

namespace social_nav2_behavior_tree
{

StopCondition::StopCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  topic_(condition_name + "/stop"),
  initialized_(false),
  stop_trigger_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), 
    "\"%s\" BtConditionNode initialized",
    condition_name.c_str());
}

void StopCondition::callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void) msg;
  stop_trigger_ = true;
}

BT::NodeStatus StopCondition::tick()
{
  if (!initialized_) {
    initialize();
  }

  if (stop_trigger_ ){
    stop_trigger_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void StopCondition::initialize()
{
  getInput("stop_topic", topic_);
  sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&StopCondition::callback, this, std::placeholders::_1));
  initialized_ = true;
}

}  // namespace social_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<social_nav2_behavior_tree::StopCondition>("StopCondition");
}
