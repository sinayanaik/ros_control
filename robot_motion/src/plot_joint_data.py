#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

def plot_joint_data(csv_file):
    df = pd.read_csv(csv_file)
    has_cartesian = all(col in df.columns for col in ['gripper_x', 'gripper_y', 'gripper_z'])
    if has_cartesian:
        fig = plt.figure(figsize=(16, 12))
        ax1 = plt.subplot(2, 2, 1)
        ax2 = plt.subplot(2, 2, 2)
        ax3 = plt.subplot(2, 2, 3)
        ax4 = plt.subplot(2, 2, 4, projection='3d')
        fig.suptitle('Joint States and Cartesian Trajectory Over Time')
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
        ax4.plot(df['gripper_x'], df['gripper_y'], df['gripper_z'], 'b-', linewidth=2, label='Gripper Trajectory')
        ax4.scatter(df['gripper_x'].iloc[0], df['gripper_y'].iloc[0], df['gripper_z'].iloc[0], color='green', s=100, label='Start', marker='o')
        ax4.scatter(df['gripper_x'].iloc[-1], df['gripper_y'].iloc[-1], df['gripper_z'].iloc[-1], color='red', s=100, label='End', marker='s')
        ax4.set_xlabel('X (m)')
        ax4.set_ylabel('Y (m)')
        ax4.set_zlabel('Z (m)')
        ax4.set_title('Gripper Cartesian Trajectory')
        ax4.legend()
        ax4.grid(True)
    else:
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
        if any(col in df.columns for col in ['gripper_x', 'gripper_y', 'gripper_z']):
            fig_cart = plt.figure(figsize=(8, 6))
            ax_cart = fig_cart.add_subplot(111, projection='3d')
            ax_cart.plot3D(df.get('gripper_x', pd.Series([0]*len(df))),
                          df.get('gripper_y', pd.Series([0]*len(df))),
                          df.get('gripper_z', pd.Series([0]*len(df))),
                          'b-', linewidth=2, label='Gripper Trajectory')
            ax_cart.set_xlabel('X (m)')
            ax_cart.set_ylabel('Y (m)')
            ax_cart.set_zlabel('Z (m)')
            ax_cart.set_title('Gripper Cartesian Trajectory')
            ax_cart.legend()
            ax_cart.grid(True)
    plt.tight_layout()
    plot_file = os.path.splitext(csv_file)[0] + '_plot.png'
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {plot_file}")
    plt.show()

def main():
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