# SECBOT_DESCRIPTION

## A ROS2 Package dedicated to the establishment of the SECBOT's URDF, main configurations, and mesh/world file storage

### Description
 - `sec_description.urdf.xacro`
   - Main URDF file called into joint state publisher. Calls all other files depending on input args
 - `robot_core_hw.xacro`
   - The main chassis and wheel setup for our hardware (2 wheel setup), will be called when `use_sim_time:=true`
 - `robot_core_sw.xacro`
   - The main chassis and wheel description for our software simulations (4 wheels), will be called when `use_sim_time:=true`
 - `ros2_control.xacro`
   - the ros2_control setup file, will be called when `use_ros2_control:=true`
 - `gazebo_control.xacro`
   - the ros2_control setup file, will be called when `use_ros2_control:=false`
 - `imu.xacro`, `lidar.xacro`, `camera.xacro`
   - Sensor model setup

### Config
  - Config files for ros2_control's [controller manager](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html) and [differential drive controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html).
  - Configuration help for ros2_control can be found [here](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html#guidelines-and-best-practices)

### Launch
 - `rsp.launch.py`
   - Reads the URDF and publishes the transforms using the [robot_state_publisher package](http://wiki.ros.org/robot_state_publisher). All other launch files, including `bringup` and `launch_sim` build off of it.
 - `display` and `rviz2` are both simply for utility, as display is to ensure the transforms are being published correctly in gazebo, and similarly in rviz2

### Models
 - Holds meshs that get imported into course worlds. Currently just the competition course.

### Worlds
 - Usable world files for `launch_sim`
