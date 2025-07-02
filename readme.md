# ROS 2 Robot Control Project

A comprehensive ROS 2 project implementing a 3-DOF robot arm with a gripper, featuring position control with position, velocity, and effort state feedback. The project demonstrates best practices in ROS 2 robot control implementation, focusing on simplicity, reliability, and maintainability.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Robot Structure](#robot-structure)
3. [Prerequisites](#prerequisites)
4. [Project Structure](#project-structure)
5. [Implementation Details](#implementation-details)
6. [Control Architecture](#control-architecture)
7. [Launch System](#launch-system)
8. [Design Decisions and Evolution](#design-decisions-and-evolution)
9. [Usage Guide](#usage-guide)
10. [Troubleshooting](#troubleshooting)

## Project Overview

### Features
- 3-DOF robot arm with gripper
- Position control with comprehensive state feedback
- Real-time position command interface
- Data logging and visualization
- Gazebo simulation integration
- RViz visualization
- Modular package structure

### Design Philosophy
The project follows these key principles:
- Simplicity: Using straightforward control interfaces
- Reliability: Robust error handling and state monitoring
- Maintainability: Clear package structure and documentation
- Extensibility: Modular design for future enhancements

## Robot Structure

### Physical Specifications
- **Base Link**: Fixed base 
- **Link 1**: First arm segment 
- **Link 2**: Second arm segment 
- **Link 3**: Third arm segment 
- **Gripper**: End effector 


### Joint Configuration
All revolute joints rotate around Y-axis with:
- Position limits: ±1.57 radians (±90 degrees)
- Effort limits: 50 N⋅m
- Velocity limits: 1.0 rad/s

## Prerequisites

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-desktop-full

# Required Packages
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-position-controllers
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install python3-pip
pip3 install pandas matplotlib numpy
```

## Implementation Details

### URDF Structure (robot.urdf.xacro)
The robot's URDF is modularized into several Xacro files for better organization:

1. **robot.urdf.xacro**: Main robot description
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="3dof_robot">
  <!-- Include modular components -->
  <xacro:include filename="$(find robot_description)/urdf/robot_materials.xacro"/>
  <xacro:include filename="$(find robot_description)/urdf/robot_gazebo.xacro"/>
  <xacro:include filename="$(find robot_description)/urdf/robot_control.xacro"/>

  <!-- Link Parameters -->
  <xacro:property name="r1" value="0.024"/>   <!-- Base link radius -->
  <xacro:property name="l1" value="0.25"/>    <!-- Base link length -->
  <xacro:property name="m1" value="0.8"/>     <!-- Base link mass -->
  <!-- ... other parameters ... -->

  <!-- Inertia Macros -->
  <xacro:macro name="inertial_cylinder_z" params="mass radius length">
    <!-- ... inertia calculations ... -->
  </xacro:macro>
```

2. **robot_control.xacro**: Hardware Interface Definition
```xml
<ros2_control name="RobotSystem" type="system">
    <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="link_1_to_link_2">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <param name="initial_position">0.0</param>
    </joint>
    <!-- ... other joints ... -->
</ros2_control>
```

### Controller Configuration (robot_controllers.yaml)
```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz
    
    arm_controller:
      type: position_controllers/JointGroupPositionController

    gripper_controller:
      type: position_controllers/JointGroupPositionController

arm_controller:
  ros__parameters:
    joints:
      - link_1_to_link_2
      - link_2_to_link_3
      - link_3_to_link_4

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
      - effort

    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
```

### Position Control Implementation (send_position.py)
```python
class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        self.position_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_controller/commands',
            10
        )
        self.joint_names = [
            'link_1_to_link_2',
            'link_2_to_link_3',
            'link_3_to_link_4'
        ]
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        t = time.time() - self.start_time
        positions = [
            0.7 * math.sin(0.25 * t),  # Base joint
            0.5 * math.sin(0.35 * t),  # Middle joint
            0.3 * math.sin(0.3 * t)    # End joint
        ]
        msg = Float64MultiArray()
        msg.data = positions
        self.position_pub.publish(msg)
```

## Launch System

### 1. Robot Launch (robot.launch.py)
```python
def generate_launch_description():
    robot_description_package = get_package_share_directory("robot_description")
    
    # Launch configuration
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(robot_description_package, "urdf", "robot.urdf.xacro")
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )
    
    # Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"])
    )
    
    # RViz visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")]
    )
```

### 2. Controllers Launch (controllers.launch.py)
```python
def generate_launch_description():
    controller_config = os.path.join(
        get_package_share_directory("robot_controller"),
        "config",
        "robot_controllers.yaml"
    )
    
    # Load joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )
    
    # Load arm and gripper controllers
    delayed_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller", "gripper_controller"]
                )
            ]
        )
    )
```

## Building from Scratch

1. **Create Workspace**:
```bash
mkdir -p ros_control/src
cd ros_control/src
```

2. **Create Packages**:
```bash
ros2 pkg create robot_description --build-type ament_cmake
ros2 pkg create robot_controller --build-type ament_cmake
ros2 pkg create robot_motion --build-type ament_cmake
```

3. **Package Dependencies**:
Add the following to each package.xml:

robot_description/package.xml:
```xml
<exec_depend>gz_ros2_control</exec_depend>
<exec_depend>ros2launch</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>rviz2</exec_depend>
```

robot_controller/package.xml:
```xml
<exec_depend>position_controllers</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>controller_manager</exec_depend>
```

robot_motion/package.xml:
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

4. **File Structure**:
```
src/
├── robot_description/
│   ├── urdf/
│   │   ├── robot.urdf.xacro
│   │   ├── robot_control.xacro
│   │   ├── robot_gazebo.xacro
│   │   └── robot_materials.xacro
│   ├── launch/
│   │   ├── robot.launch.py
│   │   ├── gazebo.launch.py
│   │   └── rviz.launch.py
│   └── config/
│       └── robot.rviz
├── robot_controller/
│   ├── config/
│   │   └── robot_controllers.yaml
│   └── launch/
│       └── controllers.launch.py
└── robot_motion/
    └── src/
        ├── send_position.py
        └── plot_joint_data.py
```

5. **Build and Source**:
```bash
cd ros_control
colcon build
source install/setup.bash
```

## Usage Guide

1. Launch robot with simulation:
```bash
ros2 launch robot_description robot.launch.py
```

2. Start controllers:
```bash
ros2 launch robot_controller controllers.launch.py
```

3. Send position commands:
```bash
ros2 run robot_motion send_position.py
```

4. Monitor joint states:
```bash
ros2 topic echo /joint_states
```

5. Visualize data:
```bash
ros2 run robot_motion plot_joint_data.py <data_file.csv>
```

## Troubleshooting

1. **Controller Startup Issues**
   - Verify interface configurations match in URDF and controller config
   - Check controller parameter file paths
   - Ensure all required dependencies are installed
   - Confirm state interfaces match hardware capabilities

2. **Position Control Issues**
   - Verify joint limits in URDF
   - Check command message format
   - Monitor joint states feedback
   - Validate controller update rate

3. **Visualization Problems**
   - Verify RViz configuration
   - Check TF tree completeness
   - Ensure robot description is properly loaded
   - Validate Gazebo simulation settings

4. **Common Error Messages**
   - "Invalid state interface": Check robot_control.xacro and robot_controllers.yaml match
   - "Controller not found": Verify controller names in launch files
   - "Failed to load controller": Check package dependencies and controller types
