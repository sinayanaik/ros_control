#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import math
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # Joint names
        self.joint_names = ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']
        
        # Motion parameters
        self.amplitudes = [
            math.radians(30),  # Base joint: 30 degrees
            math.radians(17),  # Middle joint: 17 degrees
            math.radians(23)   # End joint: 23 degrees
        ]
        self.frequencies = [0.2, 0.3, 0.25]  # Hz
        self.phases = [0.0, math.pi/2, math.pi/4]  # radians
        
        # Create timer (2 Hz update rate)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info('Trajectory publisher started')

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        
        # Calculate positions
        positions = []
        for i in range(3):
            pos = self.amplitudes[i] * math.sin(2.0 * math.pi * self.frequencies[i] * time.time())
            positions.append(float(pos))
        
        # Set positions and zero velocities
        point.positions = positions
        point.velocities = [0.0, 0.0, 0.0]  # Zero velocities at end point
        point.accelerations = []  # No acceleration needed
        point.effort = []  # No effort needed
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0
        
        # Add point to trajectory
        msg.points.append(point)
        
        # Publish trajectory
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 