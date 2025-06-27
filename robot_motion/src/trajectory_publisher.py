#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Time
import math
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # Joint names
        self.joint_names = ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']
        
        # Create timer (2 Hz update rate)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info('Trajectory publisher started')

    def timer_callback(self):
        # Create message with header
        msg = JointTrajectory()
        msg.header.stamp = Time(sec=0, nanosec=0)
        msg.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        
        # Calculate positions (simple oscillation)
        t = time.time() - self.start_time
        positions = [
            0.7 * math.sin(0.25 * t),  # Base joint
            0.5 * math.sin(0.35 * t),  # Middle joint
            0.3 * math.sin(0.3 * t)  # End joint
        ]
        
        # Set only position data since we only have position interface
        point.positions = positions
        point.time_from_start.sec = 2  # 3 seconds to reach target
        point.time_from_start.nanosec = 0
        
        # Add single point to trajectory
        msg.points = [point]
        
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