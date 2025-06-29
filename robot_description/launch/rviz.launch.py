import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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

    # Create an RViz node only
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
        rviz_node
    ]) 