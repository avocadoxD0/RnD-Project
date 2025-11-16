import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
import numpy as np

class BilateralControl(Node):
    def __init__(self):
        super().__init__("bilateral_control")

        # Safe small gains
        self.master_gain = 0.4
        self.slave_gain = 0.4

        # Subscribers
        self.master_js_sub = self.create_subscription(
            JointState, "/vx300s/joint_states", self.master_cb, 10
        )
        self.slave_js_sub = self.create_subscription(
            JointState, "/rx150/joint_states", self.slave_cb, 10
        )

        # Publishers
        self.master_cmd_pub = self.create_publisher(
            JointGroupCommand, "/vx300s/commands/joint_group", 10
        )
        self.slave_cmd_pub = self.create_publisher(
            JointGroupCommand, "/rx150/commands/joint_group", 10
        )

        # Data storage
        self.master_angles = None
        self.slave_angles = None

        # Mapping matrix (6x5 example)
        self.M = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0],  # suppressed DOF
        ])

        #the above mapping is wrong

        self.timer = self.create_timer(0.05, self.control_loop)

    def master_cb(self, msg):
        self.master_angles = np.array(msg.position[:6])  # vx300s → 6 DOF

    def slave_cb(self, msg):
        self.slave_angles = np.array(msg.position[:5])  # rx150 → 5 DOF

    def control_loop(self):
        if self.master_angles is None or self.slave_angles is None:
            return

        # MASTER → SLAVE (forward mapping)
        slave_cmd = self.M.T @ self.master_angles * self.master_gain
        self.publish_slave(slave_cmd)

        # SLAVE → MASTER (feedback mapping)
        master_cmd = self.M @ self.slave_angles * self.slave_gain
        self.publish_master(master_cmd)

    def publish_slave(self, cmd):
        msg = JointGroupCommand()
        msg.name = "arm"
        msg.cmd = cmd.tolist()
        self.slave_cmd_pub.publish(msg)

    def publish_master(self, cmd):
        msg = JointGroupCommand()
        msg.name = "arm"
        msg.cmd = cmd.tolist()
        self.master_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BilateralControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
