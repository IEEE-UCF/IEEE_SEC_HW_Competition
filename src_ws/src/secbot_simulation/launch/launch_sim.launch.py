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

    world_file_name = 'obstacles.world'
    world_file_path = os.path.join(get_package_share_directory(description_package_name), 'worlds', world_file_name)
    gazebo_model_path = os.path.join(get_package_share_directory(description_package_name), 'models')
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)

    ekf_params_file = os.path.join(get_package_share_directory(navigation_package_name), 'config', 'ekf.yaml')
    gazebo_params_path = os.path.join(get_package_share_directory(simulation_package_name),'config','gazebo_params.yaml')

    # Launch config variables specific to sim
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_robot_localization = LaunchConfiguration('use_robot_localization')
    use_world_file = LaunchConfiguration('use_world_file')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world_file = LaunchConfiguration('world_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
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

    declare_use_world_file = DeclareLaunchArgument(
        name='use_world_file',
        default_value='false',
        description='Whether to load the specified world file'
    )

    declare_use_gazebo_gui = DeclareLaunchArgument(
        name='use_gazebo_gui',
        default_value='0',
        description='Whether to launch Gazebo with or without GUI, default is without gui'
    )

    # Process launchers
    start_gazebo_world = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                condition=IfCondition(LaunchConfiguration('use_world_file')),
                launch_arguments={
                    'gui': use_gazebo_gui,
                    'world': world_file_path,
                    'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
             )

    start_gazebo_empty = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                condition=UnlessCondition(LaunchConfiguration('use_world_file')),
                launch_arguments={
                    'gui': use_gazebo_gui,
                    'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
             )

    start_robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 
                                       'use_ros2_control': use_ros2_control}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    start_spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'sec_bot'],
                        output='screen')

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
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_use_robot_localization)
    ld.add_action(declare_use_world_file)
    ld.add_action(declare_use_gazebo_gui)

    ld.add_action(set_model_path)

    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_gazebo_world)
    ld.add_action(start_gazebo_empty)
    ld.add_action(start_spawn_entity)
    ld.add_action(start_diff_drive_spawner)
    ld.add_action(start_joint_broad_spawner)
    ld.add_action(start_robot_localization)

    return ld