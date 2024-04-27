cmake_minimum_required(VERSION 3.8)
project(secbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)

#For C++ Executables
# Create an add_executable line for each c++ file you want. 
add_executable(secbot_navigation src/secbot_navigation.cpp)
#To link this executable with certain packages and dependencies:
# ament_target_dependencies(${executable_name}
#   ${dependencies}
# )
target_include_directories(secbot_navigation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(secbot_navigation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# For Python Executables
ament_python_install_package(${PROJECT_NAME})
# Add additional python file locations to make the files executable using CMake
install(PROGRAMS
  scripts/follow_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

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

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
