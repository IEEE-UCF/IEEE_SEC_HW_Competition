cmake_minimum_required(VERSION 3.8)
project(topic_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME}
  SETUP_CFG
    ${PROJECT_NAME}/setup.cfg
  SCRIPTS_DESTINATION
    lib/${PROJECT_NAME}
)

ament_auto_add_library(relay_node SHARED
  src/relay_node.cpp
  src/tool_base_node.cpp
)
target_compile_definitions(relay_node PRIVATE "TOPIC_TOOLS_BUILDING_LIBRARY")

rclcpp_components_register_nodes(relay_node "topic_tools::RelayNode")

ament_auto_add_executable(relay
  src/relay.cpp
)

target_link_libraries(relay
  relay_node
)

if(BUILD_TESTING)
  find_package(std_msgs REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_relay test/test_relay.cpp)
  target_include_directories(test_relay PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_relay
    relay_node
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
