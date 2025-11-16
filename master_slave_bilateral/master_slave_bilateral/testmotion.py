#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        super().__init__('vx300s_manual_control')
        self.sub = self.create_subscription(
            JointState,
            '/vx300s/joint_states',
            self.callback,
            10)
        self.get_logger().info("Listening to /vx300s/joint_states...")

    def callback(self, msg):
        # Just log the joint positions periodically
        pos = [round(p, 3) for p in msg.position]
        self.get_logger().info(f"Current joint positions: {pos}")

def main(args=None):
    import subprocess
    rclpy.init(args=args)

    # Launch joint_state_publisher_gui in background
    subprocess.Popen([
        'ros2', 'run', 'joint_state_publisher_gui', 'joint_state_publisher_gui'
    ])
    node = JointStateListener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
