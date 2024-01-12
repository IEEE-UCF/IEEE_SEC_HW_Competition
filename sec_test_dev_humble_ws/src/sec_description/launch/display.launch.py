import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    package_name = 'sec_description'
    file_subpath = 'description/sec_description.urdf.xml'
    final_descriptor = 'description/sec_description_final.urdf.xml'

    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Use xacro to process the file
    package_path = os.path.join(get_package_share_directory('sec_description'))
    xacro_file = os.path.join(package_path,'description','sec_description.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Configure the node
    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_config.toxml()}],
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory(package_name), 'config', 'config.rviz')]]
    )

    # Run the node
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2
    ])