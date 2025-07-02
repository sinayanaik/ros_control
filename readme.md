# ROS 2 Robot Control Project

A comprehensive ROS 2 project implementing a 3-DOF robot arm with a gripper, featuring position/effort control modes, trajectory generation, and real-time data visualization.

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
- Multiple control modes (Position/Effort)
- Real-time trajectory generation
- Data logging and visualization
- Gazebo simulation integration
- RViz visualization

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
│       └── src/           # Trajectory generators
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

- **robot_control.xacro**: Control interface definition
  ```xml
  <ros2_control name="RobotSystem" type="system">
    <joint name="link_1_to_link_2">
      <!-- Current: Effort Control -->
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
      
      <!-- For Position Control:
      <command_interface name="position"/>
      <state_interface name="position"/>
      -->
      
      <!-- For Velocity Control:
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      -->
    </joint>
  </ros2_control>
  ```

### Available Control Interfaces

1. **Position Control**
   - Command Interface: `position`
   - State Interfaces: `position`
   - Use Case: Precise positioning tasks
   - Required Changes:
     - Update URDF command_interface to "position"
     - Modify controller type to "position_controllers/JointGroupPositionController"

2. **Velocity Control**
   - Command Interface: `velocity`
   - State Interfaces: `position`, `velocity`
   - Use Case: Smooth continuous motion
   - Required Changes:
     - Update URDF command_interface to "velocity"
     - Modify controller type to "velocity_controllers/JointGroupVelocityController"

3. **Effort Control**
   - Command Interface: `effort`
   - State Interfaces: `position`, `velocity`, `effort`
   - Use Case: Force control and compliant motion
   - Required Changes:
     - Update URDF command_interface to "effort"
     - Modify controller type to "effort_controllers/JointGroupEffortController"

### 2. Robot Controller Package

1. Create the package:
```bash
ros2 pkg create robot_controller --build-type ament_cmake
```

2. Key Files:
- **robot_controllers.yaml**: Controller configuration
  ```yaml
  controller_manager:
    ros__parameters:
      update_rate: 1000  # Hz
      arm_controller:
        type: "forward_command_controller/ForwardCommandController"
  
  arm_controller:
    ros__parameters:
      joints:
        - link_1_to_link_2
        - link_2_to_link_3
        - link_3_to_link_4
      interface_name: effort
  ```
  Critical: Match interface_name with URDF control interfaces

- **controllers.launch.py**: Controller spawning
  - Loads controller configurations
  - Spawns controllers in correct order
  - Common Pitfall: Ensure joint_state_broadcaster starts first

### 3. Robot Motion Package

1. Create the package:
```bash
ros2 pkg create robot_motion --build-type ament_cmake --dependencies rclpy trajectory_msgs
```

2. Key Files:
- **trajectory_publisher.py**: Trajectory generation
  ```python
  # For effort control
  self.publisher = self.create_publisher(
      JointTrajectory, 
      '/arm_controller/joint_trajectory', 
      10
  )
  ```
  Important: Match topic names with controller type

## Control Architecture

### Forward Command Controller
- Direct effort commands
- No feedback control
- Simple but requires external feedback
- Use when: Testing basic movements, implementing custom control loops

### Joint Trajectory Controller
- Position-based control
- Built-in trajectory interpolation
- Feedback control
- Use when: Smooth trajectory following needed

## Controller Selection Guide

### Recommended Approach: Joint Trajectory Controller

When implementing robot control, the Joint Trajectory Controller is recommended over individual controllers for the following reasons:

#### Advantages
1. **Unified Interface**: One controller handles all control modes
2. **Flexibility**: Switch between control modes without changing controllers
3. **Professional Grade**: Industry-standard approach
4. **MoveIt2 Compatibility**: Works with motion planning frameworks
5. **Trajectory Support**: Handles complex multi-point trajectories
6. **Online Switching**: Change control modes during operation
7. **Resource Efficiency**: Single controller vs. three separate ones

#### Configuration Example
```yaml
arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  ros__parameters:
    joints:
      - link_1_to_link_2
      - link_2_to_link_3
      - link_3_to_link_4
    
    # Support all control interfaces
    command_interfaces:
      - position
      - velocity
      - effort
    
    # Get feedback from all interfaces
    state_interfaces:
      - position
      - velocity
      - effort
    
    # Controller parameters
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
```

### Controller Types Overview

#### 1. Basic Controllers

##### Position Controller
- **Type**: `position_controllers/JointGroupPositionController`
- **Purpose**: Direct position control
- **Use Cases**: 
  - Precise positioning
  - Point-to-point movements
  - Static pose maintenance
- **Advantages**: Simple implementation
- **Limitations**: No velocity/acceleration control

##### Velocity Controller
- **Type**: `velocity_controllers/JointGroupVelocityController`
- **Purpose**: Direct velocity control
- **Use Cases**:
  - Continuous motion
  - Speed regulation
  - Smooth trajectories
- **Advantages**: Smooth motion
- **Limitations**: No position feedback

##### Effort Controller
- **Type**: `effort_controllers/JointGroupEffortController`
- **Purpose**: Direct torque/force control
- **Use Cases**:
  - Force control
  - Compliant motion
  - Impedance control
- **Advantages**: Direct force control
- **Limitations**: Requires careful tuning

#### 2. Advanced Controllers

##### Joint Trajectory Controller
- **Type**: `joint_trajectory_controller/JointTrajectoryController`
- **Features**:
  - Multi-point trajectories
  - Smooth interpolation
  - Online trajectory updates
  - MoveIt2 compatible
- **Use Cases**: Industrial applications

##### PID Controller
- **Type**: `pid_controller/PIDController`
- **Features**:
  - Configurable gains
  - Anti-windup
  - Multiple control modes
- **Use Cases**: Precise positioning

##### Admittance Controller
- **Type**: `admittance_controller/AdmittanceController`
- **Features**:
  - Force-based control
  - Compliant behavior
  - Safety-oriented
- **Use Cases**: Human-robot interaction

### Usage Examples

1. Position Control:
```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.3, 0.2]"
```

2. Velocity Control:
```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1, 0.05, 0.02]"
```

3. Effort Control:
```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [2.0, 1.5, 1.0]"
```

## Common Pitfalls

1. **Interface Mismatch**
   - Symptom: Controller fails to start
   - Fix: Ensure URDF and controller config interfaces match

2. **Controller Spawning Order**
   - Symptom: Controllers fail to start
   - Fix: Always spawn joint_state_broadcaster first

3. **Gazebo Integration**
   - Symptom: Robot doesn't move in simulation
   - Fix: Check gz_ros2_control plugin parameters

4. **Topic Mismatch**
   - Symptom: Commands not reaching robot
   - Fix: Verify controller type and corresponding topics

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

4. Run trajectory generator:
```bash
ros2 run robot_motion trajectory_publisher.py
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
```

### Common Error Messages
1. "Invalid value set during initialization for parameter 'state_interfaces'"
   - Cause: Unsupported interface combination
   - Solution: Check supported interfaces in controller documentation

2. "Could not initialize the controller"
   - Cause: Interface mismatch or missing dependencies
   - Solution: Verify URDF and controller configuration match

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## References

- [ROS 2 Controllers Documentation](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html)
- [ros2_control Framework](https://control.ros.org/)
- [Gazebo Simulation](https://gazebosim.org/)
- [ROS 2 Control Tutorials](https://control.ros.org/master/doc/ros2_control/index.html)
