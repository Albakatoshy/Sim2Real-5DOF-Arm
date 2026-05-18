#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class ServoLogger(Node):
    def __init__(self):
        super().__init__('servo_logger')
        # Subscribe to the topic published by your joint_state_broadcaster
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info("Servo Logger started. Waiting for joint states...")

    def joint_state_callback(self, msg):
        # Clear the terminal slightly for readability (optional)
        print("\033[H\033[J", end="") 
        
        self.get_logger().info('--- Current Servo Positions ---')
        
        # Match each joint name with its position
        for name, position in zip(msg.name, msg.position):
            # Convert radians to degrees for easier reading
            degrees = math.degrees(position)
            self.get_logger().info(f"{name}: {position:.4f} rad  ({degrees:.1f}°)")

def main(args=None):
    rclpy.init(args=args)
    node = ServoLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down logger.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()