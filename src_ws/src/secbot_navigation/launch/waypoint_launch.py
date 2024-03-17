import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    waypoint_package = get_package_share_directory('secbot_navigation')

    return LaunchDescription([        
        Node(
            package='secbot_navigation',
            namespace='',
            executable='follow_waypoints.py',
            name='follow_waypoints',
        )
    ])