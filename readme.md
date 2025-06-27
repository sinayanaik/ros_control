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
│   ├── robot_controller/      # Robot control configuration
│   │   ├── config/           # Controller configurations
│   │   │   └── robot_controllers.yaml  # Controller parameters
│   │   └── launch/           # Controller launch files
│   │       └── controllers.launch.py   # Controller spawning
│   └── robot_motion/         # Trajectory generation package
│       ├── src/              # Source code directory
│       │   └── trajectory_publisher.py  # Trajectory generation node
│       ├── include/          # Header files (if needed)
│       │   └── robot_motion/
│       ├── CMakeLists.txt    # Build configuration
│       └── package.xml       # Package manifest
```

## Packages Overview

### 1. robot_description
Handles the robot's physical description and simulation setup.

### 2. robot_controller
Manages the robot's control systems and configurations.

### 3. robot_motion
The `robot_motion` package is responsible for generating continuous trajectories for the robot arm. It implements a Python-based trajectory publisher that creates smooth, sinusoidal motions for each joint using position control.

#### Key Components:
- **trajectory_publisher.py**: Main node that generates and publishes joint trajectories
  - Runs at 2 Hz update rate
  - Implements position-only control through JointTrajectory messages
  - Uses 2-second time_from_start for smooth transitions

#### Motion Parameters:
The trajectory publisher generates sinusoidal motions with the following parameters:
- **Base Joint (link_1_to_link_2)**:
  - Amplitude: 0.7 radians
  - Frequency: 0.25 Hz

- **Middle Joint (link_2_to_link_3)**:
  - Amplitude: 0.5 radians
  - Frequency: 0.35 Hz

- **End Joint (link_3_to_link_4)**:
  - Amplitude: 0.3 radians
  - Frequency: 0.3 Hz

#### Implementation Details:
- Uses ROS 2 JointTrajectory messages with position-only interface
- Sends single-point trajectories with 2-second duration
- Implements proper shutdown handling and error management
- Provides real-time trajectory generation without pre-computed paths
- Respects the controller's position-only command interface

## Key Configuration Files

### 1. robot_controllers.yaml
```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    # Available Controllers
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    gripper_controller:
      type: forward_command_controller/ForwardCommandController

# Controller-specific configurations
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
```

Common Issues:
- Update rate mismatch with Gazebo (should match or be multiple of Gazebo rate)
- Missing or incorrect joint names
- Wrong controller types for intended use
- Missing command/state interfaces

Tweaks:
- Adjust update_rate for performance vs. accuracy
- Add velocity/effort interfaces for different control modes
- Modify controller parameters for response characteristics

### 2. robot_control.xacro
Key Components:
```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSystem</plugin>
  </hardware>
  
  <joint name="link_1_to_link_2">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <param name="initial_position">0.0</param>
  </joint>
  <!-- Similar for other joints -->
</ros2_control>
```

Common Issues:
- Mismatched interface types with controllers
- Incorrect initial positions causing startup issues
- Missing or wrong plugin specifications
- Incorrect joint names

Tweaks:
- Add different control interfaces (position/velocity/effort)
- Modify initial positions
- Add joint limits and safety parameters
- Configure transmission ratios if needed

### 3. controllers.launch.py
Key Features:
```python
# Load controller configurations
controller_config = os.path.join(
    get_package_share_directory("robot_controller"),
    "config",
    "robot_controllers.yaml"
)

# Spawn controllers in sequence
spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
)

# Spawn other controllers after joint state broadcaster
delayed_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["arm_controller", "gripper_controller"],
)
```

Common Issues:
- Controllers spawning in wrong order
- Missing controller configurations
- Timing issues during startup
- Parameter file not found

Tweaks:
- Adjust controller loading sequence
- Add delay between controller spawning
- Modify controller parameters at launch
- Add condition handling for robustness

### 4. gazebo.launch.py
Key Components:
```python
# Load URDF/XACRO
robot_description = Command(['xacro ', xacro_file])

# Spawn robot state publisher
robot_state_pub = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": robot_description}]
)

# Launch Gazebo with specific world
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([gazebo_pkg, '/launch/gazebo.launch.py']),
)

# Spawn robot in Gazebo
spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=['-topic', 'robot_description', '-name', 'robot'],
)
```

Common Issues:
- URDF/XACRO parsing errors
- Missing Gazebo plugins
- Robot spawning at wrong position/orientation
- Bridge configuration issues between ROS 2 and Gazebo

Tweaks:
- Modify robot spawn position/orientation
- Change Gazebo world parameters
- Adjust physics parameters
- Configure ROS 2 - Gazebo bridge settings

## Common Troubleshooting

1. Controller Issues:
   - Check if joint_state_broadcaster is running first
   - Verify controller configurations match URDF
   - Ensure update rates are compatible
   - Check for namespace issues

2. Gazebo Problems:
   - Verify all required plugins are installed
   - Check for URDF/SDF conversion issues
   - Monitor CPU usage and adjust physics parameters
   - Verify proper bridge configuration

3. Motion Control:
   - Start with slow, simple movements
   - Verify joint limits in URDF match controllers
   - Check for singularities in robot configuration
   - Monitor controller feedback and errors

## Running the System

1. Launch Gazebo simulation:
```bash
ros2 launch robot_description gazebo.launch.py
```

2. Start controllers:
```bash
ros2 launch robot_controller controllers.launch.py
```

3. Run motion generator:
```bash
ros2 run robot_motion trajectory_publisher.py
```

## Motion Control Details

### Trajectory Generation
The trajectory publisher creates continuous, smooth motions by:
1. Computing sinusoidal positions for each joint
2. Publishing trajectories at 2 Hz
3. Using a 2-second time_from_start for smooth transitions
4. Maintaining independent frequencies for each joint

### Joint Control
Each joint is controlled independently with:
- Position-based control through ros2_control
- Continuous motion within joint limits
- Smooth transitions between setpoints
- Real-time trajectory updates

### Motion Patterns
The combined joint motions create an interesting pattern due to:
- Different frequencies creating non-repeating patterns
- Varying amplitudes providing diverse workspace coverage
- Position-only control for precise movement
- Coordinated motion through synchronized publishing

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
- Python 3.8+
- trajectory_msgs
- rclpy

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

## Detailed Configuration Guide

### 1. robot_controllers.yaml
This file defines all controller configurations and their parameters.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Control loop frequency in Hz
    
    # Define available controllers
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    gripper_controller:
      type: forward_command_controller/ForwardCommandController

# Arm controller specific configuration
arm_controller:
  ros__parameters:
    joints:
      - link_1_to_link_2    # Base joint
      - link_2_to_link_3    # Middle joint
      - link_3_to_link_4    # End joint
    
    command_interfaces:
      - position            # Control mode
    
    state_interfaces:
      - position           # Feedback mode
    
    allow_partial_joints_goal: false    # All joints must be specified in goals
    open_loop_control: true             # Don't use feedback for control
    constraints:
      stopped_velocity_tolerance: 0.01   # Maximum velocity considered as stopped
      goal_time: 0.0                    # Unlimited goal time
      link_1_to_link_2:                 # Joint-specific constraints
        trajectory: 0.1                 # Maximum trajectory deviation
        goal: 0.1                      # Goal tolerance
```

Configuration Options:
1. Controller Types:
   - `joint_trajectory_controller`: For coordinated joint motion
   - `forward_command_controller`: For simple single-command control
   - `joint_state_broadcaster`: Required for publishing joint states

2. Interface Types:
   - `position`: Position control mode
   - `velocity`: Velocity control mode
   - `effort`: Torque/force control mode

3. Performance Parameters:
   - `update_rate`: Controller update frequency
   - `constraints`: Motion and goal constraints
   - `state_publish_rate`: Feedback publication rate

### 2. robot_control.xacro
This file configures the ROS 2 Control interfaces for the robot.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="robot_control" params="prefix">
    
    <!-- ROS 2 Control configuration -->
    <ros2_control name="GazeboSystem" type="system">
      <hardware>
        <plugin>gz_ros2_control/GazeboSystem</plugin>
      </hardware>

      <!-- Base Joint Configuration -->
      <joint name="${prefix}link_1_to_link_2">
        <command_interface name="position">
          <param name="min">-1.57</param>     <!-- -90 degrees -->
          <param name="max">1.57</param>      <!-- 90 degrees -->
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>    <!-- Optional feedback -->
        <param name="initial_position">0.0</param>
      </joint>

      <!-- Middle Joint Configuration -->
      <joint name="${prefix}link_2_to_link_3">
        <command_interface name="position">
          <param name="min">-1.57</param>
          <param name="max">1.57</param>
        </command_interface>
        <state_interface name="position"/>
        <param name="initial_position">0.0</param>
      </joint>

      <!-- End Joint Configuration -->
      <joint name="${prefix}link_3_to_link_4">
        <command_interface name="position">
          <param name="min">-1.57</param>
          <param name="max">1.57</param>
        </command_interface>
        <state_interface name="position"/>
        <param name="initial_position">0.0</param>
      </joint>

      <!-- Gripper Joint Configuration -->
      <joint name="${prefix}link_4_to_gripper">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <param name="initial_position">0.0</param>
      </joint>
    </ros2_control>

    <!-- Gazebo plugin for ros2_control -->
    <gazebo>
      <plugin name="gz_ros2_control" filename="libgz_ros2_control-system.so">
        <parameters>$(find robot_controller)/config/robot_controllers.yaml</parameters>
        <robot_param>robot_description</robot_param>
        <robot_param_node>robot_state_publisher</robot_param_node>
      </plugin>
    </gazebo>

  </xacro:macro>
</robot>
```

Key Components:
1. Hardware Interface:
   - Specifies the Gazebo simulation plugin
   - Configures communication between ROS 2 and Gazebo

2. Joint Configuration:
   - Command interfaces (what you can control)
   - State interfaces (what you can measure)
   - Joint limits and initial positions
   - Safety parameters

3. Plugin Configuration:
   - Links to controller configuration
   - Sets up robot description parameters
   - Configures simulation interface

### 3. controllers.launch.py
This launch file manages controller startup and configuration.

```python
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Load controller configurations
    controller_config = os.path.join(
        get_package_share_directory("robot_controller"),
        "config",
        "robot_controllers.yaml"
    )

    # Configure Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="screen",
    )

    # Start Joint State Broadcaster first
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Start Arm Controller after Joint State Broadcaster
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Start Gripper Controller last
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Make sure controllers start in sequence
    arm_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    gripper_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_delay,
        gripper_controller_delay,
    ])
```

Key Features:
1. Controller Loading:
   - Loads controller configurations from YAML
   - Starts controller_manager node
   - Spawns controllers in correct sequence

2. Sequencing:
   - Joint State Broadcaster starts first
   - Arm Controller starts after broadcaster
   - Gripper Controller starts last
   - Uses event handlers for proper timing

3. Error Handling:
   - Waits for successful startup of each controller
   - Provides proper shutdown sequence
   - Outputs status messages for debugging
