# ROS 2 Robot Control Project

A comprehensive ROS 2 project implementing a 3-DOF robot arm with a gripper, featuring multiple control modes, continuous trajectory generation, and real-time data logging. The robot can be simulated in Gazebo with various control interfaces and includes data visualization capabilities.

## Features

- 3-DOF robot arm with gripper end-effector
- Multiple control modes: Position, Velocity, and Effort control
- Real-time control at 1000 Hz
- Continuous trajectory generation
- Gazebo simulation integration
- Modular package structure
- Python-based trajectory generation
- Real-time data logging to CSV
- Data visualization and plotting
- Comprehensive controller configurations

## Robot Specifications

- **Arm Structure:**
  - Base Link: 25 cm length, 2.4 cm radius
  - Middle Link: 15 cm length, 2.0 cm radius
  - End Link: 15 cm length, 2.0 cm radius
  - Gripper: 10 cm length, 1.5 cm radius

- **Joint Configuration:**
  - All joints: ±90 degrees range (±1.57 radians)
  - Multiple control interfaces: position, velocity, effort
  - 1000 Hz control rate
  - Real-time state feedback

## ROS 2 Controllers Overview

Based on the [ROS 2 Controllers documentation](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html), this project supports various controller types for different control requirements:

### Individual Control Controllers

#### 1. Position Controllers (`position_controllers/JointGroupPositionController`)
- **Interface**: `hardware_interface::HW_IF_POSITION`
- **Purpose**: Direct position control of joints
- **Command Type**: Single position setpoint per joint
- **Use Cases**: 
  - Precise positioning tasks
  - Point-to-point movements
  - Static pose maintenance
- **Advantages**: Simple, direct control
- **Limitations**: No velocity or acceleration control

#### 2. Velocity Controllers (`velocity_controllers/JointGroupVelocityController`)
- **Interface**: `hardware_interface::HW_IF_VELOCITY`
- **Purpose**: Direct velocity control of joints
- **Command Type**: Single velocity setpoint per joint
- **Use Cases**:
  - Continuous motion control
  - Speed regulation
  - Smooth trajectory following
- **Advantages**: Smooth motion, good for continuous operation
- **Limitations**: No position feedback in control loop

#### 3. Effort Controllers (`effort_controllers/JointGroupEffortController`)
- **Interface**: `hardware_interface::HW_IF_EFFORT`
- **Purpose**: Direct torque/force control
- **Command Type**: Single effort setpoint per joint
- **Use Cases**:
  - Force control applications
  - Compliant motion
  - Impedance control
- **Advantages**: Direct force control, good for interaction
- **Limitations**: Requires careful tuning, can be unstable

### Advanced Controllers

#### 4. Joint Trajectory Controller (`joint_trajectory_controller/JointTrajectoryController`)
- **Purpose**: Comprehensive trajectory execution
- **Supported Interfaces**: Position, Velocity, Effort
- **Features**:
  - Multi-point trajectory execution
  - Smooth interpolation between waypoints
  - Online trajectory replacement
  - Time-based trajectory following
  - Compatible with MoveIt2
- **Advantages**: 
  - Most flexible control option
  - Supports all control modes
  - Professional-grade trajectory execution
- **Use Cases**: Complex motion planning, industrial applications

#### 5. PID Controller (`pid_controller/PIDController`)
- **Purpose**: Proportional-Integral-Derivative control
- **Features**:
  - Configurable PID gains (Kp, Ki, Kd)
  - Anti-windup protection
  - Multiple control modes (position, velocity, effort)
  - Real-time parameter tuning
- **Advantages**: 
  - Precise control with feedback
  - Handles disturbances well
  - Industry standard control method
- **Use Cases**: Precise positioning, disturbance rejection

#### 6. Admittance Controller (`admittance_controller/AdmittanceController`)
- **Purpose**: Impedance/admittance control for human-robot interaction
- **Features**:
  - Virtual spring-damper system
  - Force-based motion control
  - Compliant behavior
  - Safety-oriented control
- **Advantages**:
  - Safe human-robot interaction
  - Natural force feedback
  - Adaptive behavior
- **Use Cases**: Collaborative robotics, force-sensitive tasks

### Broadcasters

#### Joint State Broadcaster (`joint_state_broadcaster/JointStateBroadcaster`)
- **Purpose**: Publishes joint states to ROS topics
- **Data Published**: Position, velocity, effort feedback
- **Topic**: `/joint_states`
- **Use Cases**: State monitoring, data logging, visualization

## Project Structure

```
ros_control/
├── src/
│   ├── robot_description/           # Robot URDF and simulation setup
│   │   ├── urdf/                   # Robot description files
│   │   │   ├── robot.urdf.xacro           # Main robot description
│   │   │   ├── robot_control.xacro        # ROS 2 Control configuration
│   │   │   ├── robot_gazebo.xacro         # Gazebo-specific configuration
│   │   │   └── robot_materials.xacro      # Visual materials definition
│   │   ├── launch/                 # Simulation launch files
│   │   │   ├── gazebo.launch.py           # Gazebo simulation launcher
│   │   │   └── rviz.launch.py             # RViz visualization launcher
│   │   └── config/                 # Configuration files
│   │       └── robot.rviz                 # RViz configuration
│   ├── robot_controller/           # Controller configuration and management
│   │   ├── config/                 # Controller configurations
│   │   │   └── robot_controllers.yaml    # Controller parameters
│   │   └── launch/                 # Controller launch files
│   │       └── controllers.launch.py     # Controller spawning
│   └── robot_motion/               # Trajectory generation and data logging
│       ├── src/                    # Python source code
│       │   ├── trajectory_publisher.py         # Position-based trajectory generator
│       │   ├── trajectory_effort_publisher.py  # Effort-based trajectory generator
│       │   └── plot_joint_data.py              # Data visualization script
│       └── data/                   # Recorded joint data (CSV files)
```

## Configuration Files Details

### 1. robot_controllers.yaml
**Location**: `src/robot_controller/config/robot_controllers.yaml`

**Purpose**: Defines all controller configurations and parameters

**Current Configuration**:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Control loop frequency in Hz

    # Controller definitions
    arm_controller:
      type: "forward_command_controller/ForwardCommandController"
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    gripper_controller:
      type: forward_command_controller/ForwardCommandController

# Arm controller specific configuration
arm_controller:
  ros__parameters:
    joints:
      - link_1_to_link_2    # Base joint
      - link_2_to_link_3    # Middle joint
      - link_3_to_link_4    # End joint
    interface_name: effort  # Control interface type
    open_loop_control: true
    allow_integration_in_goal_trajectories: true

# Gripper controller configuration
gripper_controller:
  ros__parameters:
    joints:
      - link_4_to_gripper
    interface_name: effort
    open_loop_control: true
    allow_integration_in_goal_trajectories: true
```

**Alternative Configurations**:

1. **Position Control Setup**:
```yaml
arm_controller:
  type: position_controllers/JointGroupPositionController
  interface_name: position
```

2. **Velocity Control Setup**:
```yaml
arm_controller:
  type: velocity_controllers/JointGroupVelocityController
  interface_name: velocity
```

3. **Trajectory Control Setup**:
```yaml
arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  command_interfaces: [position, velocity, effort]
  state_interfaces: [position, velocity, effort]
```

### 2. controllers.launch.py
**Location**: `src/robot_controller/launch/controllers.launch.py`

**Purpose**: Manages controller startup sequence and configuration

**Key Features**:
- Sequential controller spawning
- Event-based startup coordination
- Parameter loading from YAML
- Error handling and recovery

**Launch Sequence**:
1. Load controller configurations
2. Start joint_state_broadcaster
3. Wait for broadcaster to be ready
4. Start arm_controller and gripper_controller

### 3. gazebo.launch.py
**Location**: `src/robot_description/launch/gazebo.launch.py`

**Purpose**: Launches Gazebo simulation environment

**Components**:
- Robot state publisher
- Gazebo simulation server
- Robot model spawning
- ROS 2 - Gazebo bridge
- Clock synchronization

### 4. robot_control.xacro
**Location**: `src/robot_description/urdf/robot_control.xacro`

**Purpose**: Defines ROS 2 Control interfaces for each joint

**Interface Configuration**:
```xml
<joint name="link_1_to_link_2">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
</joint>
```

**Supported Interfaces**:
- Command interfaces: effort (can be extended to position, velocity)
- State interfaces: position, velocity, effort feedback

### 5. robot_gazebo.xacro
**Location**: `src/robot_description/urdf/robot_gazebo.xacro`

**Purpose**: Gazebo-specific configuration and plugins

**Key Components**:
- gz_ros2_control plugin configuration
- Controller parameter linking
- Simulation-specific settings

## Motion Generation Scripts

### 1. trajectory_publisher.py
**Purpose**: Position-based trajectory generation
- **Update Rate**: 2 Hz
- **Message Type**: JointTrajectory
- **Control Mode**: Position-only
- **Features**: Sinusoidal motion patterns

### 2. trajectory_effort_publisher.py
**Purpose**: Effort-based trajectory generation with data logging
- **Update Rate**: 10 Hz
- **Message Type**: Float64MultiArray
- **Control Mode**: Effort-only
- **Features**: 
  - Real-time effort feedback
  - CSV data logging
  - Sinusoidal effort patterns

### 3. plot_joint_data.py
**Purpose**: Data visualization and analysis
- **Input**: CSV files from trajectory_effort_publisher.py
- **Output**: PNG plots with position, velocity, effort data
- **Features**:
  - Multi-subplot visualization
  - Time-series analysis
  - Joint-specific data display

## Prerequisites

- ROS 2 Jazzy
- Gazebo Ignition
- Python 3.8+
- Required ROS 2 packages:
  - ros2_control
  - ros2_controllers
  - gz_ros2_control
  - trajectory_msgs
  - rclpy
- Python packages:
  - pandas
  - matplotlib

## Installation

1. Clone the repository:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
pip3 install pandas matplotlib
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Basic Operation

1. Launch Gazebo simulation:
```bash
ros2 launch robot_description gazebo.launch.py
```

2. Start the controllers:
```bash
ros2 launch robot_controller controllers.launch.py
```

3. Run trajectory generation (choose one):
   
   Position-based control:
   ```bash
   ros2 run robot_motion trajectory_publisher.py
   ```
   
   Effort-based control with data logging:
   ```bash
   ros2 run robot_motion trajectory_effort_publisher.py
   ```

### Monitoring and Visualization

4. Monitor joint states:
```bash
ros2 topic echo /joint_states
```

5. Visualize recorded data:
```bash
ros2 run robot_motion plot_joint_data.py joint_data/joint_states_YYYYMMDD_HHMMSS.csv
```

### Controller Management

6. List active controllers:
```bash
ros2 control list_controllers
```

7. Switch controller types:
```bash
ros2 control switch_controllers --start <controller_name> --stop <controller_name>
```

## Data Logging

The system automatically logs joint states to CSV files in the `joint_data/` directory. Each file contains:
- Timestamp (seconds from start)
- Position data for each joint (radians)
- Velocity data for each joint (rad/s)
- Effort data for each joint (Nm)

File naming convention: `joint_states_YYYYMMDD_HHMMSS.csv`

## Troubleshooting

### Common Issues

1. **Controller Spawning Failures**:
   - Check if joint_state_broadcaster is running first
   - Verify controller configuration matches URDF
   - Ensure update rates are compatible

2. **Gazebo Simulation Issues**:
   - Verify all required plugins are installed
   - Check for URDF/SDF conversion issues
   - Monitor CPU usage and adjust physics parameters

3. **Motion Control Problems**:
   - Start with slow, simple movements
   - Verify joint limits in URDF match controllers
   - Check for singularities in robot configuration

4. **Data Logging Issues**:
   - Check file permissions in data directory
   - Verify CSV file creation in working directory
   - Monitor disk space for large datasets

### Performance Optimization

1. **Control Rate Tuning**:
   - Adjust `update_rate` in controller configuration
   - Balance between performance and accuracy
   - Monitor CPU usage during operation

2. **Simulation Performance**:
   - Reduce physics update rate if needed
   - Use simplified collision models
   - Adjust Gazebo physics parameters

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## References

- [ROS 2 Controllers Documentation](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html)
- [ros2_control Framework](https://control.ros.org/)
- [Gazebo Simulation](https://gazebosim.org/)
- [ROS 2 Control Tutorials](https://control.ros.org/master/doc/ros2_control/index.html) 