import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MasterSlaveMirror(Node):
    def __init__(self):
        super().__init__('master_slave_mirror')

        # Subscribe to RX150 joint states (master)
        self.master_sub = self.create_subscription(
            JointState,
            '/rx150/joint_states',
            self.master_callback,
            10
        )

        # Publisher for VX300 (slave)
        self.slave_pub = self.create_publisher(
            JointState,
            '/vx300/joint_states',
            10
        )

        # VX300’s joint names (same as RX150’s)
        self.slave_joint_names = [
            'waist', 'shoulder', 'elbow',
            'wrist_angle', 'wrist_rotate',
            'gripper', 'left_finger', 'right_finger'
        ]

    def master_callback(self, msg):
        # Create a new JointState message for VX300
        inverted_msg = JointState()
        inverted_msg.header = msg.header
        inverted_msg.name = self.slave_joint_names

        # Invert all positions (reverse motion)
        inverted_msg.position = [-p for p in msg.position]

        # Optional: copy velocities & efforts if available, but inverted
        if msg.velocity:
            inverted_msg.velocity = [-v for v in msg.velocity]
        if msg.effort:
            inverted_msg.effort = [-e for e in msg.effort]

        # Publish inverted state to VX300
        self.slave_pub.publish(inverted_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MasterSlaveMirror()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
