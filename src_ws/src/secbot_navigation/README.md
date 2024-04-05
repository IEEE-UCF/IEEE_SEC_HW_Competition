# secbot_navigation


This package has six subdirectories. 

1.The config subdirectory contains configs used by the launch files of this package. Check ALL config files for sim_time parameters.If having issues, read through comments/documentation within these files. If this documentation is convoluted: determine the package you have questions about, and use the web to search "package_name params ros2". The right website may better explain different params.
    
    Files headed with "ekf" contain the parameters for the robot_localization package - started through the secbot_simulation/launch_sim.launch.py file. 
    
    The mapper_params_online_async.yaml file contains parameters for the slam_toolbox package - started through the slam_mapping.launch.py file. 
    
    The nav2_params.yaml file contains params for ALL of the Nav2, AMCL, and waypoint_action_follower packages. You likely do NOT need to change use_sim_time for any packages in the nav2_params_file, but rather in the launch files that use these packages. 


2.The launch subdirectory contains launch files.
    First Check: Check all launch files for a sim_time parameter. Enable/Disable this depending on sim/no sim. The only launch file that doesn't have this paramter might be waypoint_launch.py.
    
    Second Check: If confused, determine the package you have questions about, and use the web to search "package_name params ros2". The right website may explain different params, of which may be causing your confusion.

    rviz2.launch.py starts rviz2 with a params file thats meant to help the user with robot visualization during navigation. The configs can be easily changed using the rviz2 GUI and then clicking file/save top left.

    slam_mapping.launch.py launches the slam_toolbox package, which uses the robot's LIDAR to make a map and simultaneously references this map to remember the robots location. SLAM can also be used solely for localization, if the right params are set in the config/mapper_params_online_async.yaml file. The map can be saved - refer to the documentation of subdirectory #3 to see how. SLAM is known to be more heavy on the robot's processing. The scans that this package relies on may have early messages that send errors throughout the entire system, and this is an issue that we and Articulated Robotics were never able to fix.

    localization_launch.py launches the nav2 amcl package, which uses a pre-made map and consistent LIDAR scans to help the robot find it's location with respect to its environment. Nav2 uses data from AMCL, and fuses it with data from the odometry-based package called robot_localization to have a good idea of it's location at all times. The scans that this package relies on may have early messages that send errors throughout the entire system, and this is an issue we(and Articulated Robotics YT) were never able to fix.This launch file is launched by bringup_launch, but only in the case where you'd rather launch your localization and navigation with ONE command rather than TWO.

    navigation_launch.py launches a large number of nodes from the navigation package. This launch file needs (amcl or slam) along with robot_localization(may be started through launch_sim) to be running before it can navigate and function. This launch file is launched by bringup_launch, but only in the case where you'd rather launch your localization and navigation with ONE command rather than TWO.

    bringup_launch.py launches up to two variations of the navigation stack by launching user-specified key launch files: localization_launch.py, navigation_launch.py, or slam_mapping.launch.py. Please refer to the documentation above for explanations on the purposes of these launch files. It will first set various variables and launch conditions, such as use_sim_time and use SLAM, and will proceed to create a container with two of the three launch files(packages) specified: SLAM and Nav2 or AMCL and Nav2. This bringup is meant to launch BOTH localization and navigation together, so you don't need to launch the navigation or localization launches if you plan to use this one! You will still need odometry via the robot_localization package. This is the most consistent startup AFTER you have created a map using SLAM WITHOUT Nav2, and you can do this by launching ONLY localization.launch.py after getting the robot ready. If you intend to use amcl, this bringup_launch.py file should have SLAM disabled.

    waypoint_launch.py starts the exectuable from scripts/follow_waypoints.py. This file already has documentation, and it is the file that references the waypoint_action_follower package. The waypoint_follower_action package is also referenced during the startup of nav2 - Nav2 has a lifecycle manager that MAY EXPECT this launch file to have begun before it complete's it's startup! This launch file and the script it calls WILL EXPECT AMCL to have started before it can process and move to waypoints.

    ball_tracker_launch.py launches another launch file that's in the ball_tracker package. These launch files have various launch_description params that can be changed. Ultimately, this launch file will run three ball tracker nodes at the same time. The detect_ball node outputs the position of the ball in the camera  frame if seen, the detect_ball_3d node outputs the position of the ball with respect to the robot, and the follow_ball node outputs command velocities to the remapped "launch_argument" topic in the launch file to move the robot toward the ball. Idea #1: use a different OpenCV package to track objects other than balls, and use the calculation ability of the ball_tracker package to track various objects. Idea #2: Use the location of the ball thats published by the detect_ball_3d node and use Nav2 to navigate straight to that point. In doing so, Nav2 will avoid obstacles that the follow_ball node is too "dumb" to avoid.

3.The maps subdirectory will contain maps made by Rviz2. In order to make these maps: Run your robot with SLAM, activate the Rviz2 launch/GUI, click on plugins top-left, add the SLAM plugin to your Rviz2 options. Beside the save and the serialize map options, enter a name for your new map, and click Save Map and Serialize Map. The maps will be saved to another location on your system - copy them over to this directory.

4.The secbot_navigation subdirectory was made to help CMakeLists correctly interpret the python script used for waypoint following. It just contains a innit.py file that is MEANT to be empty.


--The files in the directories below might need to be made into executables, see the CMakeLists file to do so--

5.The scripts subdirectory will contain any python files added to this package. Try to refrain from using python scripts in this package because we've only ever used one in this fashion, and more than one may cause issues. 

6.The src subdirectory will contain files that make nodes or start processes. At the moment, this directory contains a simple file that has been made into an executable by CMakeLists, but it just prints to the terminal.
