#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
from datetime import datetime

def plot_joint_data(csv_file):
    # Read the CSV file
    df = pd.read_csv(csv_file)
    
    # Create a figure with three subplots (one for each joint)
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
        ax2 = ax.twinx()
        ax2.plot(df['timestamp'], df[f'{joint}_effort'], 
                 label='Effort (Nm)', color='red', linestyle='--')
        
        # Set labels and title
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad) / Velocity (rad/s)', color='blue')
        ax2.set_ylabel('Effort (Nm)', color='red')
        ax.set_title(title)
        
        # Add legends
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        
        # Grid
        ax.grid(True)
    
    # Adjust layout to prevent overlap
    plt.tight_layout()
    
    # Save the plot
    plot_file = os.path.splitext(csv_file)[0] + '_plot.png'
    plt.savefig(plot_file)
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