import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Load controllers configuration
    controller_config = os.path.join(
        get_package_share_directory("robot_controller"),
        "config",
        "robot_controllers.yaml"
    )

    # Load controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[controller_config],
    )

    # Delay start of robot_controller after `joint_state_broadcaster_spawner`
    delayed_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller", "gripper_controller"],
                    output="screen",
                    parameters=[controller_config],
                )
            ],
        )
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        delayed_controller_spawner,
    ])