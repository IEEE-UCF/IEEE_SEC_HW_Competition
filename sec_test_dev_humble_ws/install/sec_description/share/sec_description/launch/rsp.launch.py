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
        executable='robot_state_publisher',
        output='screen',
        arguments=[final_descriptor]
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher
    ])