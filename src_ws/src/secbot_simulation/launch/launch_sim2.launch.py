import os
import xacro

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

    ekf_params_file = os.path.join(get_package_share_directory(navigation_package_name), 'config', 'ekf.yaml')
    gazebo_params_path = os.path.join(get_package_share_directory(simulation_package_name),'config','gazebo_params.yaml')

    # Launch config variables specific to sim
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_localization = LaunchConfiguration('use_robot_localization')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_robot_localization = DeclareLaunchArgument(
        name='use_robot_localization',
        default_value='true',
        description='Use robot_localization if true'
    )

    start_diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    start_joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    start_robot_localization = Node(
        condition=IfCondition(use_robot_localization),
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_params_file]
    )

    # to control manually:
    # gazebo_control.xacro
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # ros2_control.xacro
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
    # in separate terminal

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_localization)

    ld.add_action(start_diff_drive_spawner)
    ld.add_action(start_joint_broad_spawner)
    ld.add_action(start_robot_localization)

    return ld