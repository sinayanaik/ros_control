# ROS 2 Robot Control Project

A comprehensive ROS 2 project implementing a 3-DOF robot arm with a gripper, featuring position control with position/velocity state feedback, trajectory generation, and real-time data visualization.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Robot Structure](#robot-structure)
3. [Prerequisites](#prerequisites)
4. [Project Structure](#project-structure)
5. [Step-by-Step Implementation](#step-by-step-implementation)
6. [Control Architecture](#control-architecture)
7. [Controller Selection Guide](#controller-selection-guide)
8. [Common Pitfalls](#common-pitfalls)
9. [Usage Guide](#usage-guide)
10. [Troubleshooting](#troubleshooting)

## Project Overview

### Features
- 3-DOF robot arm with gripper
- Position control with position/velocity feedback
- Real-time trajectory generation
- Data logging and visualization
- Gazebo simulation integration
- RViz visualization
- MoveIt2 compatibility

## Robot Structure

### Links
- base_link
- link_1
- link_2
- link_3
- gripper_base
- gripper_left_finger
- gripper_right_finger

### Joints
- link_1_to_link_2
- link_2_to_link_3
- link_3_to_link_4
- link_4_to_gripper
- gripper_left_finger_joint
- gripper_right_finger_joint

### Joint Connections

| Joint Name | Connects From | Connects To | Description |
|------------|--------------|-------------|-------------|
| link_1_to_link_2 | link_1 | link_2 | base rotation |
| link_2_to_link_3 | link_2 | link_3 | shoulder |
| link_3_to_link_4 | link_3 | gripper_base | elbow |
| link_4_to_gripper | gripper_base | gripper | wrist |
| gripper_left_finger_joint | gripper_base | gripper_left_finger | left finger |
| gripper_right_finger_joint | gripper_base | gripper_right_finger | right finger |

## Prerequisites

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-desktop-full

# Required Packages
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-joint-trajectory-controller
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install python3-pip
pip3 install pandas matplotlib numpy
```

## Project Structure

```
ros_control/
├── src/
│   ├── robot_description/   # Robot model and visualization
│   │   ├── urdf/           # Robot description files
│   │   │   ├── robot.urdf.xacro        # Main robot description
│   │   │   ├── robot_control.xacro     # Control interfaces
│   │   │   ├── robot_gazebo.xacro      # Gazebo configuration
│   │   │   └── robot_materials.xacro   # Visual properties
│   │   ├── launch/         # Launch files
│   │   └── config/         # RViz configuration
│   ├── robot_controller/   # Control configuration
│   │   ├── config/        # Controller parameters
│   │   └── launch/        # Controller spawning
│   └── robot_motion/      # Motion generation
│       └── src/           # Position control nodes
```

## Step-by-Step Implementation

### 1. Robot Description Package

1. Create the package:
```bash
ros2 pkg create robot_description --build-type ament_cmake
```

2. Key Files:
- **robot.urdf.xacro**: Main robot description
  - Defines physical structure
  - Includes control and Gazebo configurations
  - Critical Parameters:
    - Joint limits
    - Link dimensions
    - Inertial properties
  - Important: Define PI constant for joint rotations
    ```xml
    <xacro:property name="PI" value="3.14159265359"/>
    ```

- **robot_control.xacro**: Control interface definition
  ```xml
  <!-- ros2_control tag: Defines the control system configuration -->
  <ros2_control name="RobotSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <!-- Joint interface configuration -->
    <joint name="link_1_to_link_2">
      <!-- Command interface: What type of commands the joint accepts -->
      <command_interface name="position"/>
      
      <!-- State interfaces: What feedback the joint provides -->
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      
      <!-- Initial joint configuration -->
      <param name="initial_position">0.0</param>
    </joint>
  </ros2_control>

  <!-- Transmission tag: Maps actuator to joint -->
  <transmission name="link_1_to_link_2_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="link_1_to_link_2">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link_1_to_link_2_motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  ```

### Available Control Interfaces

1. **Position Control** (Current Implementation)
   - Command Interface: `position`
   - State Interfaces: `position`, `velocity`
   - Use Case: Precise positioning tasks
   - Required Changes:
     - Update URDF command_interface to "position"
     - Use JointTrajectoryController
     - Add transmission tags

2. **Velocity Control** (Alternative Option)
   - Command Interface: `velocity`
   - State Interfaces: `position`, `velocity`
   - Use Case: Smooth continuous motion
   - Required Changes:
     - Update URDF command_interface to "velocity"
     - Update controller configuration
     - Modify transmission interface

Note: Effort control is not supported by JointTrajectoryController in ROS 2 Jazzy.

### 2. Robot Controller Package

1. Create the package:
```bash
ros2 pkg create robot_controller --build-type ament_cmake
```

2. Dependencies:
```xml
<exec_depend>joint_trajectory_controller</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>controller_manager</exec_depend>
<exec_depend>gz_ros2_control</exec_depend>
```

3. Key Files:
- **robot_controllers.yaml**: Controller configuration
  ```yaml
  controller_manager:
    ros__parameters:
      update_rate: 1000  # Hz
      arm_controller:
        type: joint_trajectory_controller/JointTrajectoryController

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
      
      state_publish_rate: 50.0
      action_monitor_rate: 20.0
      allow_partial_joints_goal: true
  ```

### 3. Robot Motion Package

1. Create the package:
```bash
ros2 pkg create robot_motion --build-type ament_cmake --dependencies rclpy control_msgs
```

2. Key Files:
- **send_position.py**: Position control node
  ```python
  # Create action client for trajectory execution
  self.action_client = ActionClient(
      self, 
      FollowJointTrajectory, 
      '/arm_controller/follow_joint_trajectory'
  )
  
  # Send position commands
  goal_msg = FollowJointTrajectory.Goal()
  point = JointTrajectoryPoint()
  point.positions = positions
  point.velocities = [0.0] * len(self.joint_names)
  point.time_from_start = Duration(sec=2)
  ```

## Control Architecture

### Joint Trajectory Controller
- Position-based control
- Built-in trajectory interpolation
- Position and velocity feedback
- MoveIt2 compatible
- Smooth trajectory execution

### Controller Configuration
```yaml
arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
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
    
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
```

## Controller Selection Guide

### Joint Trajectory Controller Features
1. **Unified Interface**: One controller handles all control modes
2. **Flexibility**: Switch between control modes without changing controllers
3. **Professional Grade**: Industry-standard approach
4. **MoveIt2 Compatibility**: Works with motion planning frameworks
5. **Trajectory Support**: Handles complex multi-point trajectories
6. **Online Switching**: Change control modes during operation
7. **Resource Efficiency**: Single controller vs. three separate ones

### Configuration Example
```yaml
arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
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
    
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
```

## Common Pitfalls

1. **Interface Mismatch**
   - Symptom: Controller fails to start
   - Fix: Ensure URDF and controller config interfaces match
   - Note: JointTrajectoryController only supports position, velocity as state interfaces

2. **Missing PI Constant**
   - Symptom: XACRO processing fails
   - Fix: Define PI constant in URDF files
   ```xml
   <xacro:property name="PI" value="3.14159265359"/>
   ```

3. **Controller Spawning Order**
   - Symptom: Controllers fail to start
   - Fix: Always spawn joint_state_broadcaster first

4. **Action Interface Issues**
   - Symptom: Position commands not reaching robot
   - Fix: Check action server name and message types

## Usage Guide

1. Build the workspace:
```bash
colcon build
source install/setup.bash
```

2. Launch simulation:
```bash
ros2 launch robot_description robot.launch.py
```

3. Start controllers:
```bash
ros2 launch robot_controller controllers.launch.py
```

4. Run position control node:
```bash
ros2 run robot_motion send_position.py
```

## Troubleshooting

### Controller Issues
```bash
# Check controller status
ros2 control list_controllers

# Check available interfaces
ros2 control list_hardware_interfaces

# Monitor joint states
ros2 topic echo /joint_states

# Test action interface
ros2 action list
ros2 action info /arm_controller/follow_joint_trajectory
```

### Common Error Messages
1. "Invalid value set during initialization for parameter 'state_interfaces'"
   - Cause: Using unsupported state interfaces (e.g., effort)
   - Solution: Use only position and velocity state interfaces

2. "Could not initialize the controller"
   - Cause: Interface mismatch or missing dependencies
   - Solution: Verify URDF and controller configuration match

3. "name 'PI' is not defined"
   - Cause: Missing PI constant in URDF
   - Solution: Add PI constant definition in URDF files

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## References

- [ROS 2 Controllers Documentation](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html)
- [ros2_control Framework](https://control.ros.org/)
- [Gazebo Simulation](https://gazebosim.org/)
- [ROS 2 Control Tutorials](https://control.ros.org/master/doc/ros2_control/index.html)
