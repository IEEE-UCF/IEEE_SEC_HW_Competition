
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory





def generate_launch_description():
    # Get URDF via xacro
    description_package_name = 'secbot_description'
    pkg_path = os.path.join(get_package_share_directory(description_package_name))
    xacro_file = os.path.join(pkg_path,'description','sec_description.urdf.xacro')
    #use_ros2_control = LaunchConfiguration('use_ros2_control')
    #use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_content = Command(['xacro ', xacro_file])
    #robot_description_content = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    #robot_description_content = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    '''
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffdrive_arduino"), "urdf", "diffbot.urdf.xacro"]
            ),
        ]
    )
    '''
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = os.path.join(get_package_share_directory(description_package_name), 'config', 'hw_controller_config.yaml')
    '''
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diffdrive_arduino"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    '''
    '''
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diffdrive_arduino"), "rviz", "diffbot.rviz"]
    )
    '''

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
#    rviz_node = Node(
#        package="rviz2",
#        executable="rviz2",
#        name="rviz2",
#        output="log",
#        arguments=["-d", rviz_config_file],
#    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
 #   delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
 #       event_handler=OnProcessExit(
 #           target_action=joint_state_broadcaster_spawner,
 #           on_exit=[rviz_node],
 #       )
 #   )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
   #     delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
