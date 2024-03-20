import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    description_package_name = 'secbot_description'

    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true')

    declare_use_ros2_control = DeclareLaunchArgument(
            name='use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        )

    start_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(description_package_name), 'launch', 'rsp.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_ros2_control
        }.items()
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory(description_package_name), 'config', 'config.rviz')]]
    )

    # Run the node
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_ros2_control,
        start_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2
    ])