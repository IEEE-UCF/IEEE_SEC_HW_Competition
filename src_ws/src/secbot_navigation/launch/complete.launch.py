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