#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        
        # Create action client
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Joint names
        self.joint_names = [
            'link_1_to_link_2',
            'link_2_to_link_3',
            'link_3_to_link_4'
        ]
        
        # Create timer (2 Hz update rate)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_time = time.time()
        
        # Wait for action server
        self.action_client.wait_for_server()
        self.get_logger().info('Position publisher started')

    def timer_callback(self):
        # Create goal message
        goal_msg = FollowJointTrajectory.Goal()
        
        # Calculate positions (simple oscillation)
        t = time.time() - self.start_time
        positions = [
            0.7 * math.sin(0.25 * t),  # Base joint
            0.5 * math.sin(0.35 * t),  # Middle joint
            0.3 * math.sin(0.3 * t)    # End joint
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=2)  # 2 seconds to reach target
        
        # Set joint names and points in goal
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [point]
        
        # Send goal
        self.action_client.send_goal_async(goal_msg)
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