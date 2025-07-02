#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        
        # Create position command publisher
        self.position_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_controller/commands',
            10
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
        
        # Data storage
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
        
        # Create timer (2 Hz update rate)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info('Enhanced position publisher started')
        
        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def joint_state_callback(self, msg):
        """Callback to receive joint state data"""
        self.latest_joint_state = msg
        
    def forward_kinematics(self, joint_positions):
        """Corrected forward kinematics for 3-DOF planar robot arm in XZ plane (Y is always 0)"""
        l1, l2, l3 = 0.25, 0.15, 0.15  # Link lengths l1 = vertical offset
        l4 = 0.10  
        theta1, theta2, theta3 = joint_positions
        # Planar XZ kinematics (rotation about Y)
        x = l2 * math.sin(theta1) + l3 * math.sin(theta1 + theta2) + l4 * math.sin(theta1 + theta2 + theta3)
        z = l1 + l2 * math.cos(theta1) + l3 * math.cos(theta1 + theta2) + l4 * math.cos(theta1 + theta2 + theta3)
        y = 0.0
        return x, y, z
        
    def timer_callback(self):
        # Calculate desired positions (simple oscillation)
        t = time.time() - self.start_time
        desired_positions = [
            0.7 * math.sin(0.25 * t),  # link_1_to_link_2
            0.5 * math.sin(0.35 * t),  # link_2_to_link_3
            0.3 * math.sin(0.3 * t)    # link_3_to_link_4
        ]
        
        # Create and publish command message
        msg = Float64MultiArray()
        msg.data = desired_positions
        self.position_pub.publish(msg)
        
        # Calculate end effector position from desired joint positions
        end_x, end_y, end_z = self.forward_kinematics(desired_positions)
        
        # Get current timestamp
        current_time = t
        
        # Store desired data
        self.data['timestamp'].append(current_time)
        self.data['desired_positions'].append(desired_positions.copy())
        self.data['end_effector_x'].append(end_x)
        self.data['end_effector_y'].append(end_y)
        self.data['end_effector_z'].append(end_z)
        
        # Store actual data if available
        if self.latest_joint_state is not None:
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
                    # Joint not found, use default values
                    actual_positions.append(0.0)
                    velocities.append(0.0)
                    efforts.append(0.0)
        else:
            # No joint state data yet
            actual_positions = [0.0, 0.0, 0.0]
            velocities = [0.0, 0.0, 0.0]
            efforts = [0.0, 0.0, 0.0]
        
        self.data['actual_positions'].append(actual_positions)
        self.data['velocities'].append(velocities)
        self.data['efforts'].append(efforts)
        
        self.get_logger().info(f'Sent positions: {desired_positions}')
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C signal"""
        self.get_logger().info('Received interrupt signal. Saving data and creating plots...')
        self.save_data_and_plot()
        rclpy.shutdown()
        
    def save_data_and_plot(self):
        """Save data to CSV and create plots"""
        # Create test_datas directory in workspace
        workspace_root = Path(__file__).resolve().parents[4]  # Go up to workspace root
        test_datas_dir = workspace_root / 'test_datas'
        test_datas_dir.mkdir(exist_ok=True)
        
        # Prepare data for CSV
        csv_data = []
        for i in range(len(self.data['timestamp'])):
            row = {
                'timestamp': self.data['timestamp'][i],
                'desired_joint1': self.data['desired_positions'][i][0],
                'desired_joint2': self.data['desired_positions'][i][1],
                'desired_joint3': self.data['desired_positions'][i][2],
                'actual_joint1': self.data['actual_positions'][i][0],
                'actual_joint2': self.data['actual_positions'][i][1],
                'actual_joint3': self.data['actual_positions'][i][2],
                'velocity_joint1': self.data['velocities'][i][0],
                'velocity_joint2': self.data['velocities'][i][1],
                'velocity_joint3': self.data['velocities'][i][2],
                'effort_joint1': self.data['efforts'][i][0],
                'effort_joint2': self.data['efforts'][i][1],
                'effort_joint3': self.data['efforts'][i][2],
                'end_effector_x': self.data['end_effector_x'][i],
                'end_effector_y': self.data['end_effector_y'][i],
                'end_effector_z': self.data['end_effector_z'][i]
            }
            csv_data.append(row)
        
        # Save to CSV
        df = pd.DataFrame(csv_data)
        csv_file = test_datas_dir / f'robot_data_{int(time.time())}.csv'
        df.to_csv(csv_file, index=False)
        self.get_logger().info(f'Data saved to: {csv_file}')
        
        # Create plots
        self.create_plots(df, test_datas_dir)
        
    def create_plots(self, df, output_dir):
        """Create three separate plot windows (corrected for XZ plane)"""
        # Plot 1: End effector coordinates (3D and 2D)
        fig1 = plt.figure(figsize=(15, 6))
        # 3D plot (X, Z, Y)
        ax1_3d = fig1.add_subplot(121, projection='3d')
        ax1_3d.plot(df['end_effector_x'], df['end_effector_z'], df['end_effector_y'],
                    color='b', linewidth=2, label='End Effector Path')
        ax1_3d.scatter(df['end_effector_x'].iloc[0], df['end_effector_z'].iloc[0], df['end_effector_y'].iloc[0],
                       color='green', s=100, label='Start', marker='o')
        ax1_3d.scatter(df['end_effector_x'].iloc[-1], df['end_effector_z'].iloc[-1], df['end_effector_y'].iloc[-1],
                       color='red', s=100, label='End', marker='s')
        ax1_3d.set_xlabel('X (m)')
        ax1_3d.set_ylabel('Z (m)')
        ax1_3d.set_zlabel('Y (m)')
        ax1_3d.set_title('End Effector 3D Trajectory (XZ plane)')
        ax1_3d.legend()
        ax1_3d.grid(True)
        # 2D plot (X and Z over time)
        ax1_2d = fig1.add_subplot(122)
        ax1_2d.plot(df['timestamp'], df['end_effector_x'], 'r-', label='X', linewidth=2)
        ax1_2d.plot(df['timestamp'], df['end_effector_z'], 'b-', label='Z', linewidth=2)
        ax1_2d.set_xlabel('Time (s)')
        ax1_2d.set_ylabel('Position (m)')
        ax1_2d.set_title('End Effector X/Z Coordinates vs Time')
        ax1_2d.legend()
        ax1_2d.grid(True)
        plt.tight_layout()
        plot1_file = output_dir / 'end_effector_trajectory.png'
        plt.savefig(plot1_file, dpi=300, bbox_inches='tight')
        plt.show()
        
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
        plot2_file = output_dir / 'joint_positions_comparison.png'
        plt.savefig(plot2_file, dpi=300, bbox_inches='tight')
        plt.show()
        
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
        plot3_file = output_dir / 'joint_states_detailed.png'
        plt.savefig(plot3_file, dpi=300, bbox_inches='tight')
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