# ROS 2 Robot Control Project

A ROS 2 project implementing a 3-DOF robot arm with a gripper, featuring continuous trajectory generation and effort-based control. The robot can be simulated in Gazebo with real-time control and visualization.

## Features

- 3-DOF robot arm with gripper end-effector
- Real-time effort-based control at 1000 Hz
- Continuous trajectory generation
- Gazebo simulation integration
- Modular package structure
- Python-based trajectory generation

## Robot Specifications

- **Arm Structure:**
  - Base Link: 25 cm length, 2.4 cm radius
  - Middle Link: 15 cm length, 2.0 cm radius
  - End Link: 15 cm length, 2.0 cm radius
  - Gripper: 10 cm length, 1.5 cm radius

- **Joint Configuration:**
  - All joints: ±90 degrees range (±1.57 radians)
  - Effort-based control interface
  - 1000 Hz control rate

## Package Structure

```
ros_control/
├── src/
    ├── robot_description/      # Robot URDF and simulation
    │   ├── urdf/              # Robot description files
    │   ├── launch/            # Simulation launch files
    │   └── config/            # RViz configuration
    ├── robot_controller/      # Controller configuration
    │   ├── config/            # Controller parameters
    │   └── launch/            # Controller launch files
    └── robot_motion/          # Trajectory generation
        └── src/               # Python source code
```

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

## Installation

1. Clone the repository:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
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
   
   Effort-based control:
   ```bash
   ros2 run robot_motion trajectory_effort_publisher.py
   ```

4. Monitor joint states (in a new terminal):
   ```bash
   ros2 topic echo /joint_states
   ```
   This command shows real-time feedback for all joint state interfaces:
   - Position (rad)
   - Velocity (rad/s)
   - Effort (Nm)

## Trajectory Generation

The package provides two trajectory generation modes:

### 1. Position-Based Control
- Update rate: 2 Hz
- Uses JointTrajectory messages
- 2-second time_from_start for smooth transitions
- Independent joint frequencies

### 2. Effort-Based Control
- Update rate: 10 Hz
- Real-time effort feedback
- Joint-specific parameters:
  - Base: ±5 Nm at 0.5 Hz
  - Middle: ±3 Nm at 0.7 Hz
  - End: ±2 Nm at 0.9 Hz

## Controller Configuration

The robot uses effort-based control with the following setup:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    arm_controller:
      type: forward_command_controller/ForwardCommandController
      
    gripper_controller:
      type: forward_command_controller/ForwardCommandController

arm_controller:
  ros__parameters:
    joints:
      - link_1_to_link_2
      - link_2_to_link_3
      - link_3_to_link_4
    interface_name: effort

gripper_controller:
  ros__parameters:
    joints:
      - link_4_to_gripper
    interface_name: effort
```

## Troubleshooting

1. **Controller Issues:**
   - Ensure joint_state_broadcaster is running
   - Check controller configuration matches URDF
   - Verify update rates are compatible

2. **Gazebo Problems:**
   - Check required plugins are installed
   - Monitor CPU usage
   - Verify proper bridge configuration

3. **Motion Control:**
   - Start with slow movements
   - Check joint limits
   - Monitor controller feedback

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details. 