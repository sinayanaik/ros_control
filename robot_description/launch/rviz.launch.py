import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_description_package = get_package_share_directory("robot_description")
    
    # Launch configuration variables
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(robot_description_package, "urdf", "robot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(robot_description_package, "config", "robot.rviz"),
        description="Absolute path to rviz config file"
    )

    # Get URDF via xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Create a robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Create a joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Create an RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen"
    )

    # Launch Description
    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]) 