#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        
        # Create position command publisher
        self.position_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_controller/commands',
            10
        )
        
        # Joint names for reference
        self.joint_names = [
            'link_1_to_link_2',
            'link_2_to_link_3',
            'link_3_to_link_4'
        ]
        
        # Create timer (2 Hz update rate)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info('Position publisher started')

    def timer_callback(self):
        # Calculate positions (simple oscillation)
        t = time.time() - self.start_time
        positions = [
            0.7 * math.sin(0.25 * t),  # Base joint
            0.5 * math.sin(0.35 * t),  # Middle joint
            0.3 * math.sin(0.3 * t)    # End joint
        ]
        
        # Create and publish command message
        msg = Float64MultiArray()
        msg.data = positions
        self.position_pub.publish(msg)
        self.get_logger().info(f'Sent positions: {positions}')

def main():
    rclpy.init()
    node = JointPositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 