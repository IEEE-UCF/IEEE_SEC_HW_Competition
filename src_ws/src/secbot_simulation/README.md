# SECBOT_SIMULATION

## A ROS2 Package dedicated to the simulation setup of Gazebo and RViZ simulation environments

### Launch Files
### gazebo_world.launch.py

 - Specifically dedicated to the launch of world files in gazebo. 
 - World files are stored in `secbot_description/worlds`, but simply replacing the `world_file_name` variable with the name of the world file in that path will launch it
 - Command: ros2 launch gazebo_world.launch.py

### launch_sim.launch.py

 - The main launch file for full simulation testing.
 - Holds options to launch:
    1.  ros2_control
         - For control commands generation and encoder value reads
         - Uses the provided [ros2_control](https://control.ros.org/master/index.html) package
         - `use_ros2_control:=false`
         - Default `true`
    2.  robot_localization
         - In simulation, generates IMU data, as well as fuses odometry data from IMU and encoders to publish necessary `Odometry` transform for Nav2.
         - Uses the provided [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) package
         - `use_robot_localization:=false`
         - Default `true`
    3.  World Files
         - Provides the option to load the world file provided in the launch file.
         - To change launch file, change `world_file_name` in launch file
         - Default `true`
         - `use_world_file:=false`
    4.  Headless Gazebo
         - Whether to launch [Gazebo](https://gazebosim.org/home) with a GUI or headless. Good for when you need Gazebo to run to generate simulated values, but dont want the overhead of the GUI running, for example, when RViZ is also running
         - Default `true`
         - `use_gazebo_gui:=false`
    5.  sim_time
         - Whether to use_sim_time or not. Should always be true for this file
         - Default `true`
         - `use_ros2_control:=false`
 - To launch and control with gazebo_control.xacro:
   - `ros2 launch secbot_simulation launch_sim.launch.py`
   - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- To launch and control with ros2_control.xacro
  - `ros2 launch secbot_simulation launch_sim.launch.py`
  - `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped`



![]()