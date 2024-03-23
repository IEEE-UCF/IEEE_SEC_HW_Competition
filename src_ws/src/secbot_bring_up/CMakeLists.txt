cmake_minimum_required(VERSION 3.8)
project(secbot_bring_up)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(sw_launch_bringup src/sw_launch_bringup.cpp)
ament_target_dependencies(sw_launch_bringup
  rclcpp
  rcl_interfaces  # Include any external dependencies here
)

add_executable(hw_launch_bringup src/hw_launch_bringup.cpp)
ament_target_dependencies(hw_launch_bringup
  rclcpp
  rcl_interfaces  # Include any external dependencies here
)

add_executable(sw_nav_bringup src/sw_nav_bringup.cpp)
ament_target_dependencies(sw_nav_bringup
  rclcpp
  rcl_interfaces  # Include any external dependencies here
)

add_executable(hw_nav_bringup src/hw_nav_bringup.cpp)
ament_target_dependencies(hw_nav_bringup
  rclcpp
  rcl_interfaces  # Include any external dependencies here
)

add_executable(sw_waypoint_bringup src/sw_waypoint_bringup.cpp)
ament_target_dependencies(sw_waypoint_bringup
  rclcpp
  rcl_interfaces  # Include any external dependencies here
)

add_executable(hw_waypoint_bringup src/hw_waypoint_bringup.cpp)
ament_target_dependencies(hw_waypoint_bringup
  rclcpp
  rcl_interfaces  # Include any external dependencies here
)

target_include_directories(sw_launch_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sw_launch_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(hw_launch_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hw_launch_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(sw_nav_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sw_nav_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(hw_nav_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hw_nav_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(sw_waypoint_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sw_waypoint_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(hw_waypoint_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hw_waypoint_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sw_launch_bringup sw_nav_bringup sw_waypoint_bringup hw_launch_bringup hw_nav_bringup hw_waypoint_bringup
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
