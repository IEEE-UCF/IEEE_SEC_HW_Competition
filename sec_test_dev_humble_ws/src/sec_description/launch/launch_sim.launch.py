import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    package_name='sec_description'
    file_subpath = 'description/sec_description.urdf.xacro'
    robot_description = 'description/sec_description_final.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(package_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toprettyxml()

    robot_description = os.path.join(get_package_share_directory(package_name),robot_description)

    with open(robot_description, "w") as file:
        file.write(robot_description_raw)

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("sec_description"),
            "config",
            "controllers.yaml",
        ]
    )

    params = {'robot_description': robot_description_raw}

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Configure the node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params],
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'sec_bot'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # to control manually:
    # gazebo_control.xacro
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # ros2_control.xacro
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
    # in separate terminal

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
    ])