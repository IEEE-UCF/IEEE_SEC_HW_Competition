import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    lidar_package_name = 'rplidar_ros'
    imu_package_name = 'bno055'
    camera_package_name = ''

    start_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(imu_package_name), 'launch', 'bno055.launch.py')])
    )

    start_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(lidar_package_name), 'launch', 'rplidar.launch.py')])
    )

    #start_camera = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(), )])
    #)

    ld = LaunchDescription()

    ld.add_action(start_imu)
    ld.add_action(start_rplidar)
    # ld.add_action(start_camera)

    return ld
