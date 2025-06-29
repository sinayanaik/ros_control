#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
import pathlib

def plot_joint_data(csv_file):
    # Convert csv_file to absolute path if it's not already
    csv_file = os.path.abspath(csv_file)
    
    # Get the workspace root directory (this script is in src/robot_motion/src/)
    ws_root = str(pathlib.Path(__file__).resolve().parents[4])
    
    # Ensure joint_data directory exists in workspace root
    data_dir = os.path.join(ws_root, 'joint_data')
    os.makedirs(data_dir, exist_ok=True)
    
    print(f"Reading data from: {csv_file}")
    print(f"Will save plots in: {data_dir}")
    
    df = pd.read_csv(csv_file)
    has_cartesian = all(col in df.columns for col in ['gripper_x', 'gripper_y', 'gripper_z'])
    
    # Base filename without extension
    base_name = os.path.splitext(os.path.basename(csv_file))[0]
    
    if has_cartesian:
        # Create joint states and cartesian trajectory plot
        fig = plt.figure(figsize=(16, 12))
        ax1 = plt.subplot(2, 2, 1)
        ax2 = plt.subplot(2, 2, 2)
        ax3 = plt.subplot(2, 2, 3)
        ax4 = plt.subplot(2, 2, 4, projection='3d')
        fig.suptitle('Joint States and Cartesian Trajectory Over Time')
        
        # Plot joint states
        joints = ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']
        axes = [ax1, ax2, ax3]
        titles = ['Base Joint', 'Middle Joint', 'End Joint']
        
        for joint, ax, title in zip(joints, axes, titles):
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
        
        # Plot 3D trajectory
        ax4.plot(df['gripper_x'], df['gripper_y'], df['gripper_z'], 'b-', linewidth=2, label='Gripper Trajectory')
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
        
        # Save combined plot
        plt.tight_layout()
        plot_file = os.path.join(data_dir, f'{base_name}_combined.png')
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        print(f"Combined plot saved as: {plot_file}")
        
        # Create and save separate 3D scatter plot
        fig2 = plt.figure(figsize=(10, 8))
        ax3d = fig2.add_subplot(111, projection='3d')
        ax3d.plot(df['gripper_x'], df['gripper_y'], df['gripper_z'], 'b-', linewidth=2, label='Trajectory')
        ax3d.scatter(df['gripper_x'].iloc[0], df['gripper_y'].iloc[0], df['gripper_z'].iloc[0], 
                    color='green', s=100, label='Start', marker='o')
        ax3d.scatter(df['gripper_x'].iloc[-1], df['gripper_y'].iloc[-1], df['gripper_z'].iloc[-1], 
                    color='red', s=100, label='End', marker='s')
        ax3d.set_xlabel('X (m)')
        ax3d.set_ylabel('Y (m)')
        ax3d.set_zlabel('Z (m)')
        ax3d.set_title('Gripper 3D Trajectory')
        ax3d.legend()
        ax3d.grid(True)
        
        # Save 3D plot
        plt.tight_layout()
        plot_file_3d = os.path.join(data_dir, f'{base_name}_3d_scatter.png')
        plt.savefig(plot_file_3d, dpi=300, bbox_inches='tight')
        print(f"3D trajectory plot saved as: {plot_file_3d}")
        
    else:
        # Create joint states only plot
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Joint States Over Time')
        
        joints = ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']
        axes = [ax1, ax2, ax3]
        titles = ['Base Joint', 'Middle Joint', 'End Joint']
        
        for joint, ax, title in zip(joints, axes, titles):
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
        
        # Save joint states plot
        plt.tight_layout()
        plot_file = os.path.join(data_dir, f'{base_name}_joint_plot.png')
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        print(f"Joint states plot saved as: {plot_file}")
    
    plt.show()

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 plot_joint_data.py <csv_file>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    if not os.path.exists(csv_file):
        print(f"Error: File {csv_file} not found")
        sys.exit(1)
    
    try:
        plot_joint_data(csv_file)
    except Exception as e:
        print(f"Error plotting data: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main() 