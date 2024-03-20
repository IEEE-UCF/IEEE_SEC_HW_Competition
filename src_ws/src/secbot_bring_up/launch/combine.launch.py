import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    bring_up_package_name = 'secbot_bring_up'
    description_package_name = 'secbot_description'
    navigation_package_name = 'secbot_navigation'
    simulation_package_name = 'secbot_simulation'
    
    pkg_path = os.path.join(get_package_share_directory(description_package_name))

    ekf_params_file = os.path.join(get_package_share_directory(navigation_package_name), 'config', 'ekf.yaml')
    xacro_file = os.path.join(pkg_path,'description','sec_description.urdf.xacro')

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = os.path.join(get_package_share_directory(description_package_name), 'config', 'hw_controller_config.yaml')

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
        default_value='true',
        description='Use robot_localization if true'
    )
    
    start_hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(bring_up_package_name), 'launch', 'hardware.launch.py')])
    )

    start_robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 
                                       'use_ros2_control': use_ros2_control}.items()
    )

    start_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    ) 

    delayed_controller_manager = TimerAction(period=3.0, actions=[start_controller_manager])

    start_joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    start_diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"]
    )

    start_robot_localization = Node(
        condition=IfCondition(use_robot_localization),
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_params_file]
    )

    # ros2_control.xacro
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
    # in separate terminal

    return LaunchDescription([
        declare_use_sim_time_cmd, declare_use_ros2_control_cmd, declare_use_robot_localization,
        start_hw, start_robot_state_publisher, delayed_controller_manager,
        RegisterEventHandler(event_handler=OnProcessExit(target_action=start_controller_manager,
        on_exit=[start_joint_broad_spawner])),
        RegisterEventHandler(event_handler=OnProcessExit(target_action=start_joint_broad_spawner,
        on_exit=[start_diff_drive_spawner])),
        RegisterEventHandler(event_handler=OnProcessExit(target_action=start_diff_drive_spawner,
        on_exit=[start_robot_localization]))
    ])