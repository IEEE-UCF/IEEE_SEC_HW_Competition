cmake_minimum_required(VERSION 3.8)
project(arduino_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the include directory
include_directories(include third-party/include)

# Specify the source files
set(serial_SRCS third-party/src/serial.cc third-party/include/serial/serial.h 
    third-party/include/serial/v8stdint.h third-party/src/impl/list_ports/list_ports_linux.cc)
list(APPEND serial_SRCS third-party/src/impl/unix.cc)

# Create the serial library
add_library(serial ${serial_SRCS})
target_link_libraries(serial rt)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Declare a ROS 2 executable
add_executable(arduino_communication src/arduino_communication.cpp)
ament_target_dependencies(arduino_communication
  rclcpp
  std_msgs
)
target_link_libraries(arduino_communication serial)

# Install the executable and specify its dependencies for exporting
install(TARGETS
  arduino_communication
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include third-party/include)
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
