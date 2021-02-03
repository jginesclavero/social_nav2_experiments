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

#include "social_nav2_behavior_tree/behavior_tree_nodes/social_nav2_action.hpp"

namespace social_nav2_behavior_tree
{

SocialNav2Action::SocialNav2Action(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<NavigateToPose>(xml_tag_name, action_name, conf),
  CascadeLifecycleNode(action_name)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_=  std::make_shared<tf2_ros::Buffer>(node_->get_clock());;
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  chrono_pub_ = node_->create_publisher<std_msgs::msg::Float64>("time_A", 1);
  intimate_z_radius_ = 0.45;
}

void 
SocialNav2Action::on_tick()
{
  auto res = getInput<std::string>("goal").value();
  auto agent_id = getInput<std::string>("agent_id").value();
  RCLCPP_INFO(node_->get_logger(), 
    "Executing social action: %s to the agent %s", res.c_str(), agent_id.c_str());

  node_name_ = res + "_goal_updater_node";

  std::vector<rcl_interfaces::msg::Parameter> params;
  rcl_interfaces::msg::Parameter param;
  rcl_interfaces::msg::ParameterValue value;
  value.string_value = agent_id;
  param.name = "agent_id";
  param.value = value;
  params.push_back(param);
  set_goal_updater_params(node_name_, params);

  std_msgs::msg::Float64 msg;
  msg.data = now().seconds();
  chrono_pub_->publish(msg);
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    add_activation(node_name_);
  }
  goal_.pose.pose = wp_; // Sending an empty wp
  goal_.pose.header.frame_id = "map";
}

void 
SocialNav2Action::on_wait_for_result()
{  
  auto intent = config().blackboard->get<std::string>("df_intent");
  if (intent == "keep_close.action") {
    intimate_z_radius_ -= 0.1;
    update_intimate_zone(intimate_z_radius_);
    config().blackboard->set("df_intent", "");
  } else if (intent == "get_away.action") {
    intimate_z_radius_ += 0.2;
    update_intimate_zone(intimate_z_radius_);
    config().blackboard->set("df_intent", "");
  }
}

BT::NodeStatus 
SocialNav2Action::on_success()
{
  //halt();
  //trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  //result_.code = rclcpp_action::ResultCode::SUCCEEDED;
  //goal_result_available_ = true;
  //RCLCPP_WARN(node_->get_logger(), "on_success");
  return BT::NodeStatus::RUNNING;
}

void 
SocialNav2Action::set_goal_updater_params(
  std::string node_name, 
  std::vector<rcl_interfaces::msg::Parameter> params)
{
  std::string srv_name = node_name + "/set_parameters";
  goal_updater_params_client_ = create_client<SetParameters>(srv_name);
  bool is_server_ready = false;
  do {
    RCLCPP_INFO(get_logger(), "Waiting %s srv...", srv_name.c_str());
    is_server_ready =
      goal_updater_params_client_->wait_for_service(std::chrono::seconds(5));
  } while (!is_server_ready);
  
  auto request = std::make_shared<SetParameters::Request>();
  request->parameters = params;

  auto params_future_ = goal_updater_params_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(get_node_base_interface(), params_future_) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to call service %s", srv_name.c_str());
  }
}

void 
SocialNav2Action::update_intimate_zone(float new_radius)
{
  auto request = std::make_shared<SetParameters::Request>();
  rcl_interfaces::msg::Parameter param;
  rcl_interfaces::msg::ParameterValue value;
  value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  value.double_value = new_radius;
  param.name = "social_layer.intimate_z_radius";
  param.value = value;
  request->parameters.push_back(param);
  set_goal_updater_params(node_name_, request->parameters);

  std::string srv_name = "/global_costmap/global_costmap/set_parameters";
  update_params_client_ = create_client<SetParameters>(srv_name);
  bool is_server_ready = false;
  do {
    RCLCPP_INFO(get_logger(), "Waiting %s srv...", srv_name.c_str());
    is_server_ready =
      update_params_client_->wait_for_service(std::chrono::seconds(5));
  } while (!is_server_ready);
  auto params_future_ = update_params_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(get_node_base_interface(), params_future_) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to call service %s", srv_name.c_str());
  }
}

}  // namespace social_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<social_nav2_behavior_tree::SocialNav2Action>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<social_nav2_behavior_tree::SocialNav2Action>(
    "SocialNav2Action", builder);
}
