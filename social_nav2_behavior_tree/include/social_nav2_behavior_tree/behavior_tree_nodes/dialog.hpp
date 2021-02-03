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

#ifndef social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__DIALOG_HPP_
#define social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__DIALOG_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "dialogflow_ros_msgs/action/listen.hpp"

#include "social_nav2_behavior_tree/BTActionNode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace social_nav2_behavior_tree
{

class Dialog : public BtActionNode<dialogflow_ros_msgs::action::Listen>
{
public:
  explicit Dialog(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("server_name", "Action server name"),
    };
  }
private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace social_nav2_behavior_tree

#endif  // social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__DIALOG_HPP_
