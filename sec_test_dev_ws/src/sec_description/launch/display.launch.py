import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'sec_description'
    file_subpath = 'description/sec_description.urdf.xacro'
    final_descriptor = 'description/sec_description_final.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toprettyxml()

    final_descriptor = os.path.join(get_package_share_directory(pkg_name),final_descriptor)

    with open(final_descriptor, "w") as file:
        file.write(robot_description_raw)

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        output='screen',
        arguments=[final_descriptor]
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        node_executable='joint_state_publisher_gui',
    )

    node_rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        arguments=['-d', '/home/adenm/sec_test_dev_ws/src/sec_description/rviz/urdf_config.rviz']
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2
    ])