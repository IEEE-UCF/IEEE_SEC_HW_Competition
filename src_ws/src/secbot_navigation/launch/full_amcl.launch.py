import os
import xacro
import time

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    description_package_name = 'secbot_description'
    navigation_package_name = 'secbot_navigation'
    simulation_package_name = 'secbot_simulation'

    start_full_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(simulation_package_name), 'launch', 'launch_sim.launch.py')
        ])
    )

    start_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(navigation_package_name), 'launch', 'amcl.launch.py')
        ])
    )

    start_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(navigation_package_name), 'launch', 'rviz2.launch.py')
        ])
    )

    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(navigation_package_name), 'launch', 'navigation_launch.py')
        ])
    )   
    ld=LaunchDescription()
    
    ld.add_action(start_full_sim)
    ld.add_action(start_rviz2)
    ld.add_action(start_amcl)
    ld.add_action(start_nav2)

    return ld

'''
If you want to add timers between starting certain processes, use the code below.

    return LaunchDescription([
        launch.actions.TimerAction(
            period=0.0,
            actions=[start_full_sim]),
        launch.actions.TimerAction(
            period=10.0,
            actions=[ExecuteProcess(
            cmd=['ros2', 'launch', 'secbot_navigation', 'amcl.launch.py'],
            output='screen')]),        
        launch.actions.TimerAction(
            period=20.0,
            actions=[start_rviz2]),
        launch.actions.TimerAction(
            period=30.0,
            actions=[ExecuteProcess(
            cmd=['ros2', 'launch', 'secbot_navigation', 'navigation_launch.py'],
            output='screen')]),
        launch_testing.actions.ReadyToTest()
    ])
'''

"""
Follow the below steps for full launch:

Run this in 4 new tabs:
cd IEEE_SEC_HW_Competition/src_ws/
. install/setup.bash

ros2 launch secbot_simulation launch_sim.launch.py

ros2 run rviz2 rviz2 -d src/secbot_navigation/config/amcl_config.rviz --ros-args -p use_sim_time:=true

ros2 launch secbot_navigation bringup_launch.py use_sim_time:=true map:=./src/secbot_navigation/maps/obstacles_map_save.yaml

ros2 run secbot_navigation follow_waypoints.py

"""