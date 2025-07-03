# ROS 2 Planar Robot Arm Project: A Complete Guide

## Table of Contents
1. [Project Overview & Motivation](#project-overview--motivation)
2. [Design Philosophy & Challenges](#design-philosophy--challenges)
3. [System Architecture](#system-architecture)
4. [Package-by-Package Breakdown](#package-by-package-breakdown)
5. [Key Code Explanations](#key-code-explanations)
6. [Building the Project from Scratch](#building-the-project-from-scratch)
7. [Usage Guide](#usage-guide)
8. [Tinkering & Extending](#tinkering--extending)
9. [Troubleshooting](#troubleshooting)

---

## 1. Project Overview & Motivation

This project implements a **3-DOF planar robot arm with a gripper** using ROS 2, Gazebo, and RViz. It is designed as a learning and research platform for:
- Modern ROS 2 control and simulation
- Real-time feedback and logging
- Visualization and data analysis

**Historical Motivation:**
- To bridge the gap between simulation and real hardware control
- To provide a robust, extensible template for future robotics projects
- To document the journey, including mistakes, fixes, and design evolution

---

## 2. Design Philosophy & Challenges

**Philosophy:**
- **Simplicity:** Use standard ROS 2 tools and clear modular structure
- **Reliability:** Robust error handling, state monitoring, and logging
- **Maintainability:** Well-documented, easy to extend
- **Transparency:** Every design choice is explained

**Key Challenges & Solutions:**
- **Planar Kinematics:** Ensuring the robot moves in the XZ plane (not XY)
- **Accurate Feedback:** Synchronizing desired and actual joint states
- **Data Logging:** Making data collection and plotting seamless
- **Simulation-Realism:** Using Gazebo with `gz_ros2_control` for realistic hardware interface
- **User Experience:** Making the system easy to launch, monitor, and tinker with

---

## 3. System Architecture

- **robot_description:** Robot model, URDF/Xacro, simulation/visualization configs
- **robot_controller:** Controller configs, launch files for ROS 2 control
- **robot_motion:** Command publisher, data logger, and plotting tools

**Data Flow:**
1. `send_position.py` publishes desired joint positions
2. Controllers execute commands, Gazebo simulates physics
3. Joint states are published and logged
4. Data is saved and visualized for analysis

---

## 4. Package-by-Package Breakdown

### A. robot_description
- **urdf/robot.urdf.xacro:** Main robot model, includes modular Xacro files
- **urdf/robot_control.xacro:** Defines `ros2_control` hardware interface (using `gz_ros2_control`)
- **urdf/robot_gazebo.xacro:** Gazebo plugin integration
- **urdf/robot_materials.xacro:** Visual materials/colors
- **launch/robot.launch.py:** Launches robot_state_publisher, Gazebo, RViz
- **launch/gazebo.launch.py:** Launches only Gazebo simulation
- **launch/rviz.launch.py:** Launches only RViz
- **config/robot.rviz:** RViz visualization config

**Why this structure?**
- Modular Xacro makes the robot easy to extend (add links, sensors, etc.)
- Separate launch files for flexible simulation/visualization

### B. robot_controller
- **config/robot_controllers.yaml:**
  - Defines `arm_controller` (for 3 joints) and `gripper_controller` (for gripper)
  - Uses `position_controllers/JointGroupPositionController` for both
  - All joints have position, velocity, and effort interfaces
  - High update rate (1000 Hz) for smooth control
- **launch/controllers.launch.py:**
  - Spawns `joint_state_broadcaster` first, then arm and gripper controllers (using event handler for correct order)

**Why this structure?**
- Ensures controllers are loaded in the right order (avoids startup race conditions)
- YAML config is easy to edit for new controllers or tuning

### C. robot_motion
- **src/send_position.py:**
  - Publishes desired joint positions (sinusoidal trajectory)
  - Subscribes to `/joint_states` for feedback
  - Computes end-effector position using correct XZ-plane kinematics
  - Logs desired/actual positions, velocities, efforts, and end-effector coordinates
  - On Ctrl+C, saves all data to CSV and generates three detailed plots:
    1. End-effector trajectory (3D XZ, 2D X/Z vs time)
    2. Desired vs actual joint positions and errors
    3. Position, velocity, and effort for each joint
- **src/plot_joint_data.py:**
  - Standalone tool to plot joint/trajectory data from CSV

**Why this structure?**
- Separates motion logic from control/config
- Makes it easy to add new motion scripts or analysis tools

---

## 5. Key Code Explanations

### URDF/Xacro (Planar Kinematics)
- All joints rotate about the Y-axis (`axis="0 1 0"`)
- Robot moves in the XZ plane (Y is always 0)
- Forward kinematics in `send_position.py`:
  ```python
  x = l2 * sin(theta1) + l3 * sin(theta1 + theta2) + l4 * sin(theta1 + theta2 + theta3)
  z = l1 + l2 * cos(theta1) + l3 * cos(theta1 + theta2) + l4 * cos(theta1 + theta2 + theta3)
  y = 0.0
  ```

### send_position.py (Motion Publisher & Logger)
- **ROS Topics:**
  - **Publishers:**
    - `/arm_controller/commands` (Float64MultiArray): Sends desired joint positions
    - `/visualization_marker_array` (MarkerArray): Real-time end-effector trail visualization
  - **Subscribers:**
    - `/joint_states` (JointState): Receives actual joint states
  - **TF Lookups:**
    - Monitors transform from 'base_link' to 'gripper_base' for end-effector position

- **Data Collection (100Hz):**
  - Synchronized storage of:
    - Timestamps (relative to start)
    - Desired joint positions (from command generation)
    - Actual joint positions (from /joint_states)
    - Joint velocities (from /joint_states)
    - Joint efforts (from /joint_states)
    - End-effector position (from TF transform)

- **CSV File Structure (`/test_datas/robot_data_<timestamp>.csv`):**
  | Column | Source | Description |
  |--------|--------|-------------|
  | timestamp | time.time() - start_time | Elapsed time in seconds |
  | desired_joint[1-3] | Command generator | Desired position for each joint |
  | actual_joint[1-3] | /joint_states topic | Measured position for each joint |
  | velocity_joint[1-3] | /joint_states topic | Measured velocity for each joint |
  | effort_joint[1-3] | /joint_states topic | Measured effort for each joint |
  | end_effector_[x,y,z] | TF transform | End-effector position from base_link |

- **Real-time Visualization:**
  - End-effector trail using marker array:
    - Red sphere: Current position
    - Blue spheres: Past positions (5-second trail)
    - Markers update at 100Hz for smooth visualization
    - Auto-cleanup of old markers

- **Generated Plots:**
  1. **End-Effector Trajectory Plot:**
     - Left: 3D trajectory with start (green) and end (red) points
     - Right: X, Y, Z coordinates vs time
     - Equal aspect ratio for true spatial visualization
  2. **Joint Position Comparison:**
     - Three subplots (one per joint)
     - Shows desired (blue), actual (red), and error (green)
     - Helps identify tracking performance
  3. **Joint States Analysis:**
     - 3x3 grid showing:
       - Position (desired vs actual)
       - Velocity profile
       - Effort requirements
     - Useful for controller tuning and performance analysis

- **Data Synchronization:**
  - Only stores complete data points when both joint states and end-effector position are available
  - Prevents data misalignment in plots and CSV
  - Uses high-frequency transform lookups (100Hz) for smooth tracking

- **Error Handling:**
  - Graceful handling of transform lookup failures
  - Periodic logging of available TF frames for debugging
  - Proper cleanup of visualization markers

### robot_controllers.yaml (Controller Config)
- `arm_controller` for 3 actuated joints
- `gripper_controller` for gripper
- Both use position control, with state feedback (position, velocity, effort)
- High update and publish rates for smooth, real-time control

### controllers.launch.py (Controller Launch)
- Uses `OnProcessExit` to ensure `joint_state_broadcaster` starts before other controllers
- Prevents race conditions and ensures correct state feedback

---

## 6. Building the Project from Scratch

### Prerequisites
```bash
sudo apt install ros-jazzy-desktop-full
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-position-controllers ros-jazzy-joint-state-publisher-gui
sudo apt install python3-pip
pip3 install pandas matplotlib numpy
```

### Workspace & Packages
```bash
mkdir -p ros_control/src
cd ros_control/src
ros2 pkg create robot_description --build-type ament_cmake
ros2 pkg create robot_controller --build-type ament_cmake
ros2 pkg create robot_motion --build-type ament_cmake
```

### Add/Copy Files
- Place all URDF, Xacro, launch, config, and Python files as described above
- Ensure all dependencies are listed in each `package.xml`

### Build & Source
```bash
cd ~/ros_control
colcon build
source install/setup.bash
```

---

## 7. Usage Guide

### Launch Simulation & Visualization
```bash
ros2 launch robot_description robot.launch.py
```

### Start Controllers
```bash
ros2 launch robot_controller controllers.launch.py
```

### Run Motion Publisher & Logger
```bash
ros2 run robot_motion send_position.py
```
- Press Ctrl+C to save data and generate plots in `/test_datas/`

### Plot Data (Standalone)
```bash
ros2 run robot_motion plot_joint_data.py /test_datas/robot_data_<timestamp>.csv
```

### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

---

## 8. Tinkering & Extending

- **Change Trajectories:** Edit `send_position.py` to try new joint trajectories
- **Add Joints/Links:** Modify URDF/Xacro and controller configs
- **Tune Controllers:** Adjust gains and rates in `robot_controllers.yaml`
- **Add Sensors:** Extend URDF and add new ROS 2 nodes
- **Experiment with Logging:** Add more data fields or custom analysis in Python
- **Switch to Real Hardware:** Replace `gz_ros2_control` plugin with your hardware interface

**Advice:**
- Always keep URDF, controller config, and launch files in sync
- Use RViz and Gazebo for rapid feedback
- Use the plotting tools to debug and analyze performance

---

## 9. Troubleshooting

- **Controller Startup Issues:**
  - Ensure all interface names match between URDF and YAML
  - Check for typos in controller names and types
  - Confirm all dependencies are installed
- **Position Control Issues:**
  - Validate joint limits and command ranges
  - Monitor `/joint_states` for feedback
- **Visualization Problems:**
  - Check RViz config and TF tree
  - Ensure robot description is loaded
- **Data Logging/Plotting:**
  - Ensure Python dependencies are installed
  - Check `/test_datas/` for output files

---

## Why These Choices? (Historical Notes)
- **Planar XZ Kinematics:** Many beginner ROS projects use XY; we chose XZ to match the URDF and real-world tabletop arms
- **Event-Driven Controller Launch:** Avoids subtle bugs from race conditions
- **Comprehensive Logging:** Essential for debugging and research
- **Modular Xacro:** Makes the robot easy to extend and maintain
- **Standard ROS 2 Packages:** Ensures compatibility and community support

**Challenges Faced:**
- Initial confusion over planar axis (XY vs XZ) led to incorrect kinematics and plots; fixed by careful URDF/code review
- Controller startup order caused missing feedback; solved with event-driven launch
- Data logging and plotting required careful synchronization of desired/actual states

---

**This README is designed to be a living document. If you extend or modify the project, update this file to help the next user (or your future self)!**
