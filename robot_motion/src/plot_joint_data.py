#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
from datetime import datetime

def plot_joint_data(csv_file):
    # Read the CSV file
    df = pd.read_csv(csv_file)
    
    # Check if cartesian coordinates are available
    has_cartesian = all(col in df.columns for col in ['gripper_x', 'gripper_y', 'gripper_z'])
    
    if has_cartesian:
        # Create a figure with four subplots (three joints + cartesian)
        fig = plt.figure(figsize=(16, 12))
        
        # Create subplots: 2x2 grid
        ax1 = plt.subplot(2, 2, 1)  # Base joint
        ax2 = plt.subplot(2, 2, 2)  # Middle joint
        ax3 = plt.subplot(2, 2, 3)  # End joint
        ax4 = plt.subplot(2, 2, 4, projection='3d')  # Cartesian trajectory
        
        fig.suptitle('Joint States and Cartesian Trajectory Over Time')
        
        # Plot data for each joint
        joints = ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']
        axes = [ax1, ax2, ax3]
        titles = ['Base Joint', 'Middle Joint', 'End Joint']
        
        for joint, ax, title in zip(joints, axes, titles):
            # Plot position
            ax.plot(df['timestamp'], df[f'{joint}_position'], 
                    label='Position (rad)', color='blue')
            
            # Plot velocity on same axis
            ax.plot(df['timestamp'], df[f'{joint}_velocity'], 
                    label='Velocity (rad/s)', color='green')
            
            # Create second y-axis for effort
            ax2_twin = ax.twinx()
            ax2_twin.plot(df['timestamp'], df[f'{joint}_effort'], 
                         label='Effort (Nm)', color='red', linestyle='--')
            
            # Set labels and title
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad) / Velocity (rad/s)', color='blue')
            ax2_twin.set_ylabel('Effort (Nm)', color='red')
            ax.set_title(title)
            
            # Add legends
            lines1, labels1 = ax.get_legend_handles_labels()
            lines2, labels2 = ax2_twin.get_legend_handles_labels()
            ax2_twin.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
            
            # Grid
            ax.grid(True)
        
        # Plot 3D cartesian trajectory
        ax4.plot3D(df['gripper_x'], df['gripper_y'], df['gripper_z'], 
                   'b-', linewidth=2, label='Gripper Trajectory')
        
        # Mark start and end points
        ax4.scatter(df['gripper_x'].iloc[0], df['gripper_y'].iloc[0], df['gripper_z'].iloc[0], 
                   color='green', s=100, label='Start', marker='o')
        ax4.scatter(df['gripper_x'].iloc[-1], df['gripper_y'].iloc[-1], df['gripper_z'].iloc[-1], 
                   color='red', s=100, label='End', marker='s')
        
        ax4.set_xlabel('X (m)')
        ax4.set_ylabel('Y (m)')
        ax4.set_zlabel('Z (m)')
        ax4.set_title('Gripper Cartesian Trajectory')
        ax4.legend()
        ax4.grid(True)
        
    else:
        # Fallback to original 3-joint plot if no cartesian data
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Joint States Over Time')
        
        # Plot data for each joint
        joints = ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']
        axes = [ax1, ax2, ax3]
        titles = ['Base Joint', 'Middle Joint', 'End Joint']
        
        for joint, ax, title in zip(joints, axes, titles):
            # Plot position
            ax.plot(df['timestamp'], df[f'{joint}_position'], 
                    label='Position (rad)', color='blue')
            
            # Plot velocity on same axis
            ax.plot(df['timestamp'], df[f'{joint}_velocity'], 
                    label='Velocity (rad/s)', color='green')
            
            # Create second y-axis for effort
            ax2_twin = ax.twinx()
            ax2_twin.plot(df['timestamp'], df[f'{joint}_effort'], 
                         label='Effort (Nm)', color='red', linestyle='--')
            
            # Set labels and title
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad) / Velocity (rad/s)', color='blue')
            ax2_twin.set_ylabel('Effort (Nm)', color='red')
            ax.set_title(title)
            
            # Add legends
            lines1, labels1 = ax.get_legend_handles_labels()
            lines2, labels2 = ax2_twin.get_legend_handles_labels()
            ax2_twin.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
            
            # Grid
            ax.grid(True)
    
    # Adjust layout to prevent overlap
    plt.tight_layout()
    
    # Save the plot
    plot_file = os.path.splitext(csv_file)[0] + '_plot.png'
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {plot_file}")
    
    # Show the plot
    plt.show()

def main():
    # Check if CSV file is provided as argument
    if len(sys.argv) != 2:
        print("Usage: python3 plot_joint_data.py <csv_file>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    if not os.path.exists(csv_file):
        print(f"Error: File {csv_file} not found")
        sys.exit(1)
    
    plot_joint_data(csv_file)

if __name__ == '__main__':
    main() 