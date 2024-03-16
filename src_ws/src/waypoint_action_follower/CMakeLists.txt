cmake_minimum_required(VERSION 3.5)
project(waypoint_action_follower)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(waypoint_interfaces REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name waypoint_follower)

add_executable(${executable_name}
  src/main.cpp
)

add_executable(client
  src/waypoint_index_client.cpp
)

target_include_directories(waypoint_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/waypoint_follower.cpp
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  nav_msgs
  nav2_msgs
  nav2_util
  tf2_ros
  nav2_core
  pluginlib
  geometry_msgs
  waypoint_interfaces
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

ament_target_dependencies(client
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(wait_at_waypoint SHARED plugins/wait_at_waypoint.cpp)
ament_target_dependencies(wait_at_waypoint ${dependencies})

add_library(turn_robot_at_waypoint SHARED plugins/turn_robot_at_waypoint.cpp)
ament_target_dependencies(turn_robot_at_waypoint ${dependencies})

add_library(service_at_waypoint SHARED plugins/service_at_waypoint.cpp)
ament_target_dependencies(service_at_waypoint ${dependencies})

add_library(task_at_waypoint SHARED plugins/task_at_waypoint.cpp)
ament_target_dependencies(task_at_waypoint ${dependencies})

rclcpp_components_register_nodes(${library_name} "nav2_waypoint_follower::WaypointFollower")

install(TARGETS ${library_name} wait_at_waypoint turn_robot_at_waypoint service_at_waypoint task_at_waypoint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(wait_at_waypoint turn_robot_at_waypoint service_at_waypoint task_at_waypoint ${library_name})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(waypoint_action_follower plugins.xml)


ament_package()

