#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import time
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
import signal
import sys
from pathlib import Path
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        
        # Create position command publisher
        self.position_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_controller/commands',
            10
        )
        
        # Create marker publisher with reliable QoS
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint names for reference
        self.joint_names = [
            'link_1_to_link_2',
            'link_2_to_link_3',
            'link_3_to_link_4'
        ]
        
        # Data storage for plotting
        self.data = {
            'timestamp': [],
            'desired_positions': [],
            'actual_positions': [],
            'velocities': [],
            'efforts': [],
            'end_effector_x': [],
            'end_effector_y': [],
            'end_effector_z': []
        }
        
        # Joint state data
        self.latest_joint_state = None
        self.latest_end_effector_pos = None
        
        # Create TF buffer with a longer cache time
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create timers (2 Hz for commands, 100 Hz for markers)
        self.command_timer = self.create_timer(0.5, self.command_timer_callback)
        self.marker_timer = self.create_timer(0.01, self.marker_timer_callback)  # 100 Hz for smoother visualization
        
        self.start_time = time.time()
        self.get_logger().info('Enhanced position publisher started')
        
        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Store past positions with timestamps for trail effect
        self.ee_past_positions = []  # Store (Point, timestamp)
        self.marker_array = MarkerArray()
        self.next_marker_id = 0
        
        # Configuration for visualization
        self.TRAIL_DURATION = 5.0  # Trail duration in seconds
        self.MARKER_SIZE = 0.015   # Smaller markers for denser trail
        
        # Debug counter for transform availability
        self.transform_check_counter = 0
        self.transform_check_interval = 100  # Log every 100 attempts

    def get_end_effector_position(self):
        """Get the current end effector position from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'gripper_base',  # Track the gripper base frame
                Time(),
                Duration(seconds=0.05)  # Shorter timeout for higher frequency
            )
            
            # Create point from transform
            pt = Point()
            pt.x = transform.transform.translation.x
            pt.y = transform.transform.translation.y
            pt.z = transform.transform.translation.z
            
            self.latest_end_effector_pos = pt
            return pt
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
               tf2_ros.ExtrapolationException) as e:
            self.transform_check_counter += 1
            if self.transform_check_counter % self.transform_check_interval == 0:
                self.get_logger().warn(f'Transform error: {str(e)}')
                # Log available frames for debugging
                frames = self.tf_buffer.all_frames_as_string()
                self.get_logger().info(f'Available frames:\n{frames}')
            return None

    def joint_state_callback(self, msg):
        """Callback to receive joint state data"""
        self.latest_joint_state = msg

    def command_timer_callback(self):
        """Timer callback for sending joint commands"""
        t = time.time() - self.start_time
        desired_positions = [
            0.7 * math.sin(1 * t),  # link_1_to_link_2
            0.5 * math.cos(0.5 * t),  # link_2_to_link_3
            0.3 * math.sin(0.25 * t)    # link_3_to_link_4
        ]
        
        msg = Float64MultiArray()
        msg.data = desired_positions
        self.position_pub.publish(msg)
        
        # Get current end effector position
        ee_pos = self.get_end_effector_position()
        
        # Only store data if we have both joint states and end effector position
        if self.latest_joint_state is not None and ee_pos is not None:
            # Store timestamp and desired positions
            self.data['timestamp'].append(t)
            self.data['desired_positions'].append(desired_positions)
            
            # Store end effector position
            self.data['end_effector_x'].append(ee_pos.x)
            self.data['end_effector_y'].append(ee_pos.y)
            self.data['end_effector_z'].append(ee_pos.z)
            
            # Store joint states
            actual_positions = []
            velocities = []
            efforts = []
            
            for joint_name in self.joint_names:
                try:
                    idx = self.latest_joint_state.name.index(joint_name)
                    actual_positions.append(self.latest_joint_state.position[idx])
                    velocities.append(self.latest_joint_state.velocity[idx])
                    efforts.append(self.latest_joint_state.effort[idx])
                except ValueError:
                    actual_positions.append(0.0)
                    velocities.append(0.0)
                    efforts.append(0.0)
            
            self.data['actual_positions'].append(actual_positions)
            self.data['velocities'].append(velocities)
            self.data['efforts'].append(efforts)

    def marker_timer_callback(self):
        """High-frequency timer callback for smooth marker updates"""
        try:
            now = self.get_clock().now()
            ee_pos = self.get_end_effector_position()
            
            if ee_pos is None:
                return
                
            # Add new position to history with timestamp
            self.ee_past_positions.append((ee_pos, now.nanoseconds))
            
            # Remove positions older than TRAIL_DURATION seconds
            current_time = now.nanoseconds
            self.ee_past_positions = [(p, t) for p, t in self.ee_past_positions 
                                    if (current_time - t) < self.TRAIL_DURATION * 1e9]
            
            # Create marker array for trail visualization
            marker_array = MarkerArray()
            
            # Add markers for trail
            for i, (past_pt, ts) in enumerate(self.ee_past_positions):
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = now.to_msg()
                marker.ns = "end_effector_trail"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # Set position
                marker.pose.position = past_pt
                marker.pose.orientation.w = 1.0
                
                # Set size (small spheres)
                marker.scale.x = self.MARKER_SIZE
                marker.scale.y = self.MARKER_SIZE
                marker.scale.z = self.MARKER_SIZE
                
                # Set color (red for current position, blue for trail)
                if i == len(self.ee_past_positions) - 1:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                else:
                    # Fade from blue to transparent based on age
                    age_factor = 1.0 - (current_time - ts) / (self.TRAIL_DURATION * 1e9)
                    marker.color.r = 0.0
                    marker.color.g = 0.7
                    marker.color.b = 1.0
                    marker.color.a = 0.8 * age_factor
                
                # Set lifetime
                marker.lifetime = Duration(seconds=self.TRAIL_DURATION).to_msg()
                
                marker_array.markers.append(marker)
            
            # Delete old markers
            if len(marker_array.markers) < len(self.marker_array.markers):
                for i in range(len(marker_array.markers), len(self.marker_array.markers)):
                    delete_marker = Marker()
                    delete_marker.header.frame_id = "base_link"
                    delete_marker.header.stamp = now.to_msg()
                    delete_marker.ns = "end_effector_trail"
                    delete_marker.id = i
                    delete_marker.action = Marker.DELETE
                    marker_array.markers.append(delete_marker)
            
            # Update marker array and publish
            self.marker_array = marker_array
            self.marker_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f'Marker error: {str(e)}')

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C signal"""
        self.get_logger().info('Received interrupt signal. Saving data and creating plots...')
        self.save_data_and_plot()
        rclpy.shutdown()
        
    def save_data_and_plot(self):
        """Save data to CSV and create plots"""
        self.get_logger().info('Saving data and creating plots...')
        
        # Create test_datas directory in workspace
        workspace_root = Path(__file__).resolve().parents[4]  # Go up to workspace root
        test_datas_dir = workspace_root / 'test_datas'
        test_datas_dir.mkdir(exist_ok=True)
        
        # Convert data to DataFrame
        df = pd.DataFrame({
            'timestamp': self.data['timestamp'],
            'desired_joint1': [pos[0] for pos in self.data['desired_positions']],
            'desired_joint2': [pos[1] for pos in self.data['desired_positions']],
            'desired_joint3': [pos[2] for pos in self.data['desired_positions']],
            'actual_joint1': [pos[0] for pos in self.data['actual_positions']],
            'actual_joint2': [pos[1] for pos in self.data['actual_positions']],
            'actual_joint3': [pos[2] for pos in self.data['actual_positions']],
            'velocity_joint1': [vel[0] for vel in self.data['velocities']],
            'velocity_joint2': [vel[1] for vel in self.data['velocities']],
            'velocity_joint3': [vel[2] for vel in self.data['velocities']],
            'effort_joint1': [eff[0] for eff in self.data['efforts']],
            'effort_joint2': [eff[1] for eff in self.data['efforts']],
            'effort_joint3': [eff[2] for eff in self.data['efforts']],
            'end_effector_x': self.data['end_effector_x'],
            'end_effector_y': self.data['end_effector_y'],
            'end_effector_z': self.data['end_effector_z']
        })
        
        # Save to CSV
        timestamp = int(time.time())
        csv_file = test_datas_dir / f'robot_data_{timestamp}.csv'
        df.to_csv(csv_file, index=False)
        self.get_logger().info(f'Data saved to: {csv_file}')
        
        # Create plots
        self.create_plots(df, test_datas_dir, timestamp)
        
    def create_plots(self, df, output_dir, timestamp):
        """Create three separate plot windows"""
        # Plot 1: End effector coordinates (3D and 2D)
        fig1 = plt.figure(figsize=(15, 6))
        
        # 3D trajectory
        ax1_3d = fig1.add_subplot(121, projection='3d')
        ax1_3d.plot(df['end_effector_x'], df['end_effector_y'], df['end_effector_z'],
                    'b-', linewidth=2, label='End Effector Path')
        ax1_3d.scatter(df['end_effector_x'].iloc[0], df['end_effector_y'].iloc[0], df['end_effector_z'].iloc[0],
                       color='green', s=100, label='Start', marker='o')
        ax1_3d.scatter(df['end_effector_x'].iloc[-1], df['end_effector_y'].iloc[-1], df['end_effector_z'].iloc[-1],
                       color='red', s=100, label='End', marker='s')
        ax1_3d.set_xlabel('X (m)')
        ax1_3d.set_ylabel('Y (m)')
        ax1_3d.set_zlabel('Z (m)')
        ax1_3d.set_title('End Effector 3D Trajectory')
        ax1_3d.legend()
        ax1_3d.grid(True)
        
        # Set equal aspect ratio for 3D plot
        max_range = np.array([
            df['end_effector_x'].max() - df['end_effector_x'].min(),
            df['end_effector_y'].max() - df['end_effector_y'].min(),
            df['end_effector_z'].max() - df['end_effector_z'].min()
        ]).max() / 2.0
        
        mean_x = df['end_effector_x'].mean()
        mean_y = df['end_effector_y'].mean()
        mean_z = df['end_effector_z'].mean()
        ax1_3d.set_xlim(mean_x - max_range, mean_x + max_range)
        ax1_3d.set_ylim(mean_y - max_range, mean_y + max_range)
        ax1_3d.set_zlim(mean_z - max_range, mean_z + max_range)
        
        # 2D coordinates over time
        ax1_2d = fig1.add_subplot(122)
        ax1_2d.plot(df['timestamp'], df['end_effector_x'], 'r-', label='X', linewidth=2)
        ax1_2d.plot(df['timestamp'], df['end_effector_y'], 'g-', label='Y', linewidth=2)
        ax1_2d.plot(df['timestamp'], df['end_effector_z'], 'b-', label='Z', linewidth=2)
        ax1_2d.set_xlabel('Time (s)')
        ax1_2d.set_ylabel('Position (m)')
        ax1_2d.set_title('End Effector Coordinates vs Time')
        ax1_2d.legend()
        ax1_2d.grid(True)
        
        plt.tight_layout()
        plot1_file = output_dir / f'end_effector_trajectory_{timestamp}.png'
        plt.savefig(plot1_file)
        
        # Plot 2: Desired vs Actual joint positions and errors
        fig2, axes = plt.subplots(3, 1, figsize=(12, 10))
        joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
        
        for i, (ax, joint_name) in enumerate(zip(axes, joint_names)):
            desired_col = f'desired_joint{i+1}'
            actual_col = f'actual_joint{i+1}'
            
            ax.plot(df['timestamp'], df[desired_col], 'b-', label='Desired', linewidth=2)
            ax.plot(df['timestamp'], df[actual_col], 'r--', label='Actual', linewidth=2)
            
            # Calculate and plot error
            error = df[desired_col] - df[actual_col]
            ax.plot(df['timestamp'], error, 'g:', label='Error', linewidth=1.5)
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad)')
            ax.set_title(f'{joint_name}: Desired vs Actual Positions')
            ax.legend()
            ax.grid(True)
        
        plt.tight_layout()
        plot2_file = output_dir / f'joint_positions_comparison_{timestamp}.png'
        plt.savefig(plot2_file)
        
        # Plot 3: Position, velocity, and effort over time
        fig3, axes = plt.subplots(3, 3, figsize=(15, 12))
        
        for i in range(3):
            joint_name = f'Joint {i+1}'
            
            # Position
            ax_pos = axes[0, i]
            ax_pos.plot(df['timestamp'], df[f'desired_joint{i+1}'], 'b-', label='Desired', linewidth=2)
            ax_pos.plot(df['timestamp'], df[f'actual_joint{i+1}'], 'r--', label='Actual', linewidth=2)
            ax_pos.set_xlabel('Time (s)')
            ax_pos.set_ylabel('Position (rad)')
            ax_pos.set_title(f'{joint_name} Position')
            ax_pos.legend()
            ax_pos.grid(True)
            
            # Velocity
            ax_vel = axes[1, i]
            ax_vel.plot(df['timestamp'], df[f'velocity_joint{i+1}'], 'g-', linewidth=2)
            ax_vel.set_xlabel('Time (s)')
            ax_vel.set_ylabel('Velocity (rad/s)')
            ax_vel.set_title(f'{joint_name} Velocity')
            ax_vel.grid(True)
            
            # Effort
            ax_eff = axes[2, i]
            ax_eff.plot(df['timestamp'], df[f'effort_joint{i+1}'], 'm-', linewidth=2)
            ax_eff.set_xlabel('Time (s)')
            ax_eff.set_ylabel('Effort (N⋅m)')
            ax_eff.set_title(f'{joint_name} Effort')
            ax_eff.grid(True)
        
        plt.tight_layout()
        plot3_file = output_dir / f'joint_states_detailed_{timestamp}.png'
        plt.savefig(plot3_file)
        
        # Show all plots
        plt.show()
        
        self.get_logger().info(f'Plots saved to: {output_dir}')

def main():
    rclpy.init()
    node = JointPositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 