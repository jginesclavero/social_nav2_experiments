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

#ifndef social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__SOCIALNAV2ACTION_HPP_
#define social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__SOCIALNAV2ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "social_nav2_behavior_tree/BTActionNode.hpp"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "std_msgs/msg/float64.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using SetParameters = rcl_interfaces::srv::SetParameters;

namespace social_nav2_behavior_tree
{

class SocialNav2Action 
: public BtActionNode<NavigateToPose>,
  public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  explicit SocialNav2Action(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_wait_for_result() override;

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::string>("goal"),
      BT::InputPort<std::string>("agent_id"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> chrono_pub_;
  std::shared_ptr<rclcpp::Client<SetParameters>> goal_updater_params_client_;
  std::shared_ptr<rclcpp::Client<SetParameters>> update_params_client_;
  geometry_msgs::msg::Pose wp_;
  float intimate_z_radius_, personal_z_radius_;
  std::string node_name_;
  
  void set_goal_updater_params(
    std::string node_name, 
    std::vector<rcl_interfaces::msg::Parameter> params);
  void update_intimate_zone(float new_radius);
};

}  // namespace social_nav2_behavior_tree

#endif  // social_nav2_behavior_tree__BEHAVIOR_TREE_NODES__SOCIALNAV2ACTION_HPP_
