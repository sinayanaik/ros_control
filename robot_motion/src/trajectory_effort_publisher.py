#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time
import math
import time
import csv
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pathlib
import numpy as np

class TrajectoryEffortPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_effort_publisher')
        
        # Create publisher for effort commands
        self.publisher = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        
        # Create publisher for cartesian position marker
        self.marker_publisher = self.create_publisher(Marker, '/gripper_position_marker', 10)
        
        # Create subscriber for joint states to monitor actual efforts
        self.state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Setup TF2 listener for cartesian position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Joint names for reference
        self.joint_names = ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']
        
        # Create timer (10 Hz update rate)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        
        # Store current joint states
        self.current_positions = [0.0] * 3
        self.current_velocities = [0.0] * 3
        self.current_efforts = [0.0] * 3
        
        # Store cartesian position
        self.cartesian_position = [0.0, 0.0, 0.0]
        self.marker_id = 0
        
        try:
            # Setup CSV logging
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            # Always create data directory in joint_data under the workspace root
            ws_root = pathlib.Path(__file__).resolve().parents[2]
            self.data_dir = os.path.join(ws_root, 'joint_data')
            os.makedirs(self.data_dir, exist_ok=True)
            self.csv_file = os.path.join(self.data_dir, f'joint_states_{timestamp}.csv')
            self.get_logger().info(f'Will write data to: {self.csv_file}')
            
            # Create CSV file with headers
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                headers = ['timestamp']
                for joint in self.joint_names:
                    headers.extend([f'{joint}_position', f'{joint}_velocity', f'{joint}_effort'])
                headers.extend(['gripper_x', 'gripper_y', 'gripper_z'])
                writer.writerow(headers)
                self.get_logger().info('Successfully created CSV file with headers')
            
            self.get_logger().info('Effort trajectory publisher started')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup data logging: {str(e)}')
            self.get_logger().error(f'Exception type: {type(e).__name__}')
            self.csv_file = None

    def get_cartesian_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'gripper_base', Time())
            position = transform.transform.translation
            self.cartesian_position = [position.x, position.y, position.z]
            return True
        except Exception as e:
            self.get_logger().warn(f'Could not get cartesian position: {str(e)}')
            self.cartesian_position = [0.0, 0.0, 0.0]
            return False

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gripper_position"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.cartesian_position[0]
        marker.pose.position.y = self.cartesian_position[1]
        marker.pose.position.z = self.cartesian_position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.lifetime.sec = 5
        marker.lifetime.nanosec = 0
        self.marker_publisher.publish(marker)
        self.get_logger().info(f'Published marker id {self.marker_id} at: ({marker.pose.position.x:.3f}, {marker.pose.position.y:.3f}, {marker.pose.position.z:.3f})')
        self.marker_id += 1
        if self.marker_id > 10000:
            self.marker_id = 0

    def joint_state_callback(self, msg):
        try:
            # Update current states for monitored joints
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.current_positions[i] = msg.position[idx] if idx < len(msg.position) else 0.0
                    self.current_velocities[i] = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
                    self.current_efforts[i] = msg.effort[idx] if idx < len(msg.effort) else 0.0
            # Get cartesian position
            self.get_cartesian_position()
            # Log data to CSV if file was created successfully
            if self.csv_file:
                with open(self.csv_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    data = [time.time() - self.start_time]  # timestamp
                    for i in range(3):
                        data.extend([
                            self.current_positions[i],
                            self.current_velocities[i],
                            self.current_efforts[i]
                        ])
                    data.extend(self.cartesian_position)  # cartesian coordinates
                    writer.writerow(data)
        except Exception as e:
            self.get_logger().error(f'Error in joint state callback: {str(e)}')
            self.get_logger().error(f'Exception type: {type(e).__name__}')

    def timer_callback(self):
        try:
            msg = Float64MultiArray()
            t = time.time() - self.start_time
            efforts = [
                2.0 * math.sin(0.5 * t),
                1.5 * math.sin(0.7 * t),
                1.0 * math.sin(0.9 * t)
            ]
            msg.data = efforts
            self.publisher.publish(msg)
            self.get_cartesian_position()
            self.publish_marker()
            self.get_logger().info(
                f'Current efforts: Base: {self.current_efforts[0]:.2f} Nm, ' +
                f'Middle: {self.current_efforts[1]:.2f} Nm, ' +
                f'End: {self.current_efforts[2]:.2f} Nm, ' +
                f'Gripper pos: ({self.cartesian_position[0]:.3f}, {self.cartesian_position[1]:.3f}, {self.cartesian_position[2]:.3f})'
            )
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')
            self.get_logger().error(f'Exception type: {type(e).__name__}')

def plot_joint_data(csv_file):
    df = pd.read_csv(csv_file)
    has_cartesian = all(col in df.columns for col in ['gripper_x', 'gripper_y', 'gripper_z'])
    # Joint state plot (positions, velocities, efforts)
    fig1, axes = plt.subplots(3, 1, figsize=(12, 10))
    ax1, ax2, ax3 = axes
    fig1.suptitle('Joint States Over Time')
    joints = ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']
    axes_list = [ax1, ax2, ax3]
    titles = ['Base Joint', 'Middle Joint', 'End Joint']
    for joint, ax, title in zip(joints, axes_list, titles):
        ax.plot(df['timestamp'], df[f'{joint}_position'], label='Position (rad)', color='blue')
        ax.plot(df['timestamp'], df[f'{joint}_velocity'], label='Velocity (rad/s)', color='green')
        ax2_twin = ax.twinx()
        ax2_twin.plot(df['timestamp'], df[f'{joint}_effort'], label='Effort (Nm)', color='red', linestyle='--')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad) / Velocity (rad/s)', color='blue')
        ax2_twin.set_ylabel('Effort (Nm)', color='red')
        ax.set_title(title)
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2_twin.get_legend_handles_labels()
        ax2_twin.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        ax.grid(True)
    fig1.tight_layout()
    plot_file1 = os.path.join(str(pathlib.Path(csv_file).parent), os.path.splitext(os.path.basename(csv_file))[0] + '_joint_plot.png')
    fig1.savefig(plot_file1, dpi=300, bbox_inches='tight')
    print(f"Joint plot saved as: {plot_file1}")
    # Gripper coordinate plots
    if has_cartesian:
        # 3D trajectory plot (line)
        fig2 = plt.figure(figsize=(8, 6))
        ax3d = fig2.add_subplot(111, projection='3d')
        ax3d.plot(df['gripper_x'], df['gripper_y'], df['gripper_z'], 'b-', linewidth=2, label='Gripper Trajectory')
        ax3d.scatter(df['gripper_x'].iloc[0], df['gripper_y'].iloc[0], df['gripper_z'].iloc[0], color='green', s=100, label='Start', marker='o')
        ax3d.scatter(df['gripper_x'].iloc[-1], df['gripper_y'].iloc[-1], df['gripper_z'].iloc[-1], color='red', s=100, label='End', marker='s')
        ax3d.set_xlabel('X (m)')
        ax3d.set_ylabel('Y (m)')
        ax3d.set_zlabel('Z (m)')
        ax3d.set_title('Gripper Cartesian Trajectory (3D)')
        ax3d.legend()
        ax3d.grid(True)
        fig2.tight_layout()
        plot_file2 = os.path.join(str(pathlib.Path(csv_file).parent), os.path.splitext(os.path.basename(csv_file))[0] + '_gripper_3d_plot.png')
        fig2.savefig(plot_file2, dpi=300, bbox_inches='tight')
        print(f"Gripper 3D plot saved as: {plot_file2}")
        plt.show()
    else:
        plt.show()

def main():
    rclpy.init()
    node = None
    try:
        node = TrajectoryEffortPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        # Plot the data if CSV file exists
        if node and hasattr(node, 'csv_file') and node.csv_file and os.path.exists(node.csv_file):
            print(f"Plotting data from: {node.csv_file}")
            plot_joint_data(node.csv_file)

if __name__ == '__main__':
    main() 