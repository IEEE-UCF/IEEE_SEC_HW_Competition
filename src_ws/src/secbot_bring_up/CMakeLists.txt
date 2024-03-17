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

add_executable(gazebo_bringup src/gazebo_bringup.cpp)
ament_target_dependencies(gazebo_bringup
  rclcpp
  rcl_interfaces  # Include any external dependencies here
)

add_executable(nav_bringup src/nav_bringup.cpp)
ament_target_dependencies(nav_bringup
  rclcpp
  rcl_interfaces  # Include any external dependencies here
)

add_executable(waypoint_bringup src/waypoint_bringup.cpp)
ament_target_dependencies(waypoint_bringup
  rclcpp
  rcl_interfaces  # Include any external dependencies here
)

target_include_directories(gazebo_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gazebo_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(nav_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nav_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(waypoint_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(waypoint_bringup PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS gazebo_bringup nav_bringup waypoint_bringup
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
