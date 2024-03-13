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

    description_package_name = 'secbot_description'
    navigation_package_name = 'secbot_navigation'
    simulation_package_name = 'secbot_simulation'

    ekf_params_file = os.path.join(get_package_share_directory(navigation_package_name), 'config', 'ekf.yaml')

    # Launch config variables specific to sim
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_robot_localization = LaunchConfiguration('use_robot_localization')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'
    )

    declare_use_robot_localization = DeclareLaunchArgument(
        name='use_robot_localization',
        default_value='false',
        description='Use robot_localization if true'
    )

    start_robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 
                                       'use_ros2_control': use_ros2_control}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(description_package_name), 'config', 'hw_controller_config.yaml')

    start_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                      controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[start_controller_manager])

    start_diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"]
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager,
            on_start=[start_diff_drive_spawner]
        )
    )

    start_joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"]
    )

    delayed_joint_state_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager,
            on_start=[start_joint_broad_spawner]
        )
    )

    start_robot_localization = Node(
        condition=IfCondition(use_robot_localization),
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_params_file]
    )

    start_imu

    start_lidar

    start_camera

    # ros2_control.xacro
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
    # in separate terminal

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_use_robot_localization)

    ld.add_action(start_robot_state_publisher)
    # ld.add_action(start_robot_localization)
    ld.add_action(delayed_controller_manager)
    # ld.add_action(delayed_diff_drive_spawner)
    # ld.add_action(delayed_joint_state_spawner)

    return ld