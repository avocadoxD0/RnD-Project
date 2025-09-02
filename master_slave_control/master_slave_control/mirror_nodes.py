import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MasterSlaveMirror(Node):
    def __init__(self):
        super().__init__('master_slave_mirror')

        # Subscribe to RX150 joint states
        self.master_sub = self.create_subscription(
            JointState,
            '/rx150/joint_states',
            self.master_callback,
            10
        )

        # Publisher for VX300
        self.slave_pub = self.create_publisher(
            JointTrajectory,
            '/vx300/commands/joint_group_position',
            10
        )

        # Store VX300’s joint names (same as RX150’s)
        self.slave_joint_names = [
            'waist', 'shoulder', 'elbow',
            'wrist_angle', 'wrist_rotate',
            'gripper', 'left_finger', 'right_finger'
        ]

    def master_callback(self, msg):
        # Forward RX150 joint positions to VX300
        slave_command = JointTrajectory()
        slave_command.joint_names = self.slave_joint_names
        point = JointTrajectoryPoint()
        point.positions = list(msg.position)[:len(self.slave_joint_names)]
        point.time_from_start.sec = 1
        slave_command.points = [point]

        self.slave_pub.publish(slave_command)

def main(args=None):
    rclpy.init(args=args)
    node = MasterSlaveMirror()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
