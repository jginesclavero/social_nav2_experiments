cmake_minimum_required(VERSION 3.5)
project(social_nav2_behavior_tree)

set(CMAKE_BUILD_TYPE DEBUG)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(dialogflow_ros_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(std_srvs REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_cascade_lifecycle
  ros2_knowledge_graph
  ament_index_cpp
  nav2_behavior_tree
  nav2_msgs
  nav2_util
  dialogflow_ros_msgs
  behaviortree_cpp_v3
  std_srvs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

add_library(social_nav2_behavior_tree_navigate_to_wp_bt_node SHARED
  src/behavior_tree_nodes/navigate_to_wp.cpp
)
list(APPEND plugin_libs social_nav2_behavior_tree_navigate_to_wp_bt_node)

add_library(social_nav2_behavior_tree_social_nav2_action_bt_node SHARED
  src/behavior_tree_nodes/social_nav2_action.cpp
)
list(APPEND plugin_libs social_nav2_behavior_tree_social_nav2_action_bt_node)

add_library(social_nav2_behavior_tree_dialog_bt_node SHARED
  src/behavior_tree_nodes/dialog.cpp
)
list(APPEND plugin_libs social_nav2_behavior_tree_dialog_bt_node)

add_library(social_nav2_behavior_tree_stop_condition_bt_node SHARED
  src/behavior_tree_nodes/stop_condition.cpp
)
list(APPEND plugin_libs social_nav2_behavior_tree_stop_condition_bt_node)

add_library(social_nav2_behavior_tree_chrono_decorator_bt_node SHARED
  src/behavior_tree_nodes/chrono.cpp
)
list(APPEND plugin_libs social_nav2_behavior_tree_chrono_decorator_bt_node)

add_library(social_nav2_behavior_tree_turn_agent_decorator_bt_node SHARED
  src/behavior_tree_nodes/turn_agent.cpp
)
list(APPEND plugin_libs social_nav2_behavior_tree_turn_agent_decorator_bt_node)

add_library(social_nav2_behavior_tree_write_csv_decorator_bt_node SHARED
  src/behavior_tree_nodes/write_csv.cpp
)
list(APPEND plugin_libs social_nav2_behavior_tree_write_csv_decorator_bt_node)

add_library(social_nav2_behavior_tree_simple_goal_reached_condition_bt_node SHARED
  src/behavior_tree_nodes/simple_goal_reached_condition.cpp
)
list(APPEND plugin_libs social_nav2_behavior_tree_simple_goal_reached_condition_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(social_nav2_hri src/social_nav2_hri.cpp)
ament_target_dependencies(social_nav2_hri ${dependencies})
target_link_libraries(social_nav2_hri)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  #add_subdirectory(test)
endif()

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  social_nav2_hri
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
