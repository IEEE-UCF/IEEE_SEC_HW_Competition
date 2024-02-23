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

    navigation_package_name = 'secbot_navigation'
    simulation_package_name = 'secbot_simulation'

    start_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(simulation_package_name), 'launch', 'launch_sim.launch.py')
        ])
    )
    start_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(navigation_package_name), 'launch', 'rviz2.launch.py')
        ])
    )
    start_full_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(navigation_package_name), 'launch', 'bringup_launch.py')
        ])
    )

    return LaunchDescription([
        launch.actions.TimerAction(
        period=0.0,
        actions=[start_simulation]),
        launch.actions.TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
        cmd=['ros2', 'launch', 'secbot_navigation', 'bringup_launch.py'],
        output='screen')]),
        launch.actions.TimerAction(
        period=17.0,
        actions=[start_rviz2]),
    launch_testing.actions.ReadyToTest()
    ])

'''
        launch.actions.TimerAction(
        period=25.0,
        actions=[start_waypoint_follow]),
    ld=LaunchDescription()
    ExecuteProcess(
        cmd=['ros2', 'launch', 'secbot_navigation', 'bringup_launch.py'],
        output='screen')
    launch.actions.TimerAction(
        period=0.0,
        actions=[ld.add_action(start_simulation)])
#    launch.actions.TimerAction(
#        period=15.0,
#        actions=[ld.add_action(start_full_navigation)])

    return ld

'''
'''    start_waypoint_follow = Node(
            package='secbot_navigation',
            namespace='',
            executable='follow_waypoints.py',
            name='follow_waypoints',
        )
'''
'''
Follow the below steps for full launch:

Run this in 4 new tabs:
cd IEEE_SEC_HW_Competition/src_ws/
. install/setup.bash

ros2 launch secbot_simulation launch_sim.launch.py

ros2 run rviz2 rviz2 -d src/secbot_navigation/config/amcl_config.rviz --ros-args -p use_sim_time:=true

ros2 launch secbot_navigation bringup_launch.py use_sim_time:=true map:=./src/secbot_navigation/maps/obstacles_map_save.yaml

ros2 run secbot_navigation follow_waypoints.py
'''