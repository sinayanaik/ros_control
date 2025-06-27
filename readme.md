# ROS 2 Robot Control Project

This project implements a 3-DOF robot arm with a gripper using ROS 2 and Gazebo Ignition simulation. The robot is controlled using ros2_control and can be simulated in Gazebo.

## Project Structure

```
ros_control/
├── src/
│   ├── robot_description/      # Robot URDF and simulation setup
│   │   ├── urdf/              # Robot description files
│   │   │   ├── robot.urdf.xacro        # Main robot description
│   │   │   ├── robot_control.xacro     # ROS 2 Control configuration
│   │   │   ├── robot_gazebo.xacro      # Gazebo-specific configuration
│   │   │   └── robot_materials.xacro   # Visual materials definition
│   │   └── launch/            # Launch files
│   │       └── gazebo.launch.py        # Gazebo simulation launcher
│   └── robot_controller/      # Robot control configuration
│       ├── config/           # Controller configurations
│       │   └── robot_controllers.yaml  # Controller parameters
│       └── launch/           # Controller launch files
│           └── controllers.launch.py   # Controller spawning
```

## XACRO Organization

The robot description is modularly organized using XACRO files:

1. `robot.urdf.xacro`: Main robot description file that:
   - Defines the robot's physical structure
   - Includes all other XACRO files
   - Defines joint and link properties
   - Specifies robot dimensions and inertial properties

2. `robot_control.xacro`: ROS 2 Control configuration
   - Defines the control interfaces for each joint
   - Configures hardware interfaces (position control)
   - Sets up joint limits and control parameters

3. `robot_gazebo.xacro`: Gazebo simulation settings
   - Configures Gazebo plugins
   - Sets up physics properties
   - Links to controller configurations

4. `robot_materials.xacro`: Visual properties
   - Defines colors and materials for visualization
   - Sets up visual properties for each link

## Gazebo Spawn Process

The robot spawning in Gazebo is handled by `gazebo.launch.py` and follows these steps:

1. Launch Gazebo simulation environment
2. Load and parse XACRO files into URDF
3. Start Robot State Publisher
4. Spawn robot model in Gazebo using ROS 2 control
5. Setup ROS 2 - Gazebo communication bridge

```python
# Launch Gazebo with robot
ros2 launch robot_description gazebo.launch.py
```

## Controller Configuration

Controllers are managed separately from the Gazebo simulation:

1. Controller Types:
   - Joint State Broadcaster: Publishes joint states
   - Arm Controller: Joint Trajectory Controller for the 3-DOF arm
   - Gripper Controller: Forward Command Controller for the gripper

2. Configuration (`robot_controllers.yaml`):
   ```yaml
   controller_manager:
     update_rate: 1000  # Hz
     arm_controller:
       type: joint_trajectory_controller/JointTrajectoryController
     gripper_controller:
       type: forward_command_controller/ForwardCommandController
   ```

3. Controller Activation:
   ```python
   # Launch controllers
   ros2 launch robot_controller controllers.launch.py
   ```

## Usage Examples

1. Control the Robot Arm:
```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {
    stamp: {sec: 0, nanosec: 0}
  },
  joint_names: ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4'],
  points: [
    {
      positions: [0.5, 0.5, -0.5],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}"
```

2. Control the Gripper:
```bash
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"
```

## Robot Specifications

- 3 Degrees of Freedom (DOF) arm
- Position-controlled joints
- Gripper end-effector
- Joint Limits: ±1.57 radians (±90 degrees)
- Control Rate: 1000 Hz
- Simulation Environment: Gazebo Ignition

## Dependencies

- ROS 2 (Tested on ROS 2 Jazzy)
- Gazebo Ignition
- ros2_control
- ros2_controllers
- gz_ros2_control

## Building and Running

1. Clone the repository:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
```

3. Source the workspace:
```bash
source install/setup.bash
```

4. Launch the simulation:
```bash
ros2 launch robot_description gazebo.launch.py
```

5. In a new terminal, launch the controllers:
```bash
ros2 launch robot_controller controllers.launch.py
```

## Additional Information

- The robot uses position control for all joints
- Controllers are spawned sequentially to ensure proper initialization
- The simulation runs at 1000 Hz for smooth motion
- The robot model includes proper inertial properties for realistic simulation
