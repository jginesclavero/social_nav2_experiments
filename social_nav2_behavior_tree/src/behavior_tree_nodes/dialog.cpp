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

#include "social_nav2_behavior_tree/behavior_tree_nodes/dialog.hpp"

namespace social_nav2_behavior_tree
{

Dialog::Dialog(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<dialogflow_ros_msgs::action::Listen>(xml_tag_name, action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

void Dialog::on_tick()
{  
  RCLCPP_INFO(node_->get_logger(), "Calling dialog...");
}

BT::NodeStatus Dialog::on_success()
{
  config().blackboard->set("df_intent", result_.result.get()->result.intent);
  RCLCPP_INFO(node_->get_logger(), "Intent: %s", result_.result.get()->result.intent.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace social_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<social_nav2_behavior_tree::Dialog>(
        name, "dialogflow_client/listen", config);
    };

  factory.registerBuilder<social_nav2_behavior_tree::Dialog>(
    "Dialog", builder);
}
