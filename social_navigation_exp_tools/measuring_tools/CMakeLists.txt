cmake_minimum_required(VERSION 3.5)
project(measuring_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)


set(dependencies
    rclcpp
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2
    nav2_costmap_2d
    nav_msgs
    pedsim_msgs
)

include_directories(include)

add_executable(dummy_tf2_node src/dummy_tf2_node.cpp)
ament_target_dependencies(dummy_tf2_node ${dependencies})

add_executable(distance_to_agent_node src/distance_to_agent.cpp)
ament_target_dependencies(distance_to_agent_node ${dependencies})

add_executable(robot_cost_node src/robot_cost_pub.cpp)
ament_target_dependencies(robot_cost_node ${dependencies})

add_executable(path_pub_node src/path_pub.cpp)
ament_target_dependencies(path_pub_node ${dependencies})

install(TARGETS
  dummy_tf2_node
  distance_to_agent_node
  robot_cost_node
  path_pub_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
