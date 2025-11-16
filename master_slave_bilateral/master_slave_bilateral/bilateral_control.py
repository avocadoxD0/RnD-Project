#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
import numpy as np
import time


class SimpleBilateral(Node):
    def __init__(self):
        super().__init__("simple_bilateral_control")

        # ----------------------
        # Gains & Update Rate
        # ----------------------
        self.speed_scale = 1.2          # How fast slave follows master
        self.master_stiffness = 0.03    # Very soft – allows hand movement
        self.timer_dt = 0.008           # 125 Hz → Very responsive

        # ----------------------
        # Joint counts
        # ----------------------
        self.nM = 5   # rx150: waist, shoulder, elbow, wrist_angle, wrist_rotate
        self.nS = 6   # vx300s: waist, shoulder, elbow, roll, wrist_angle, wrist_rotate

        # ----------------------
        # Mapping matrix (6×5)
        # ----------------------
        # Slave joints:
        # 0 waist
        # 1 shoulder
        # 2 elbow
        # 3 forearm_roll   (NO mapping from master)
        # 4 wrist_angle
        # 5 wrist_rotate
        #
        # Master joints:
        # 0 waist
        # 1 shoulder
        # 2 elbow
        # 3 wrist_angle
        # 4 wrist_rotate
        #
        # So row 3 of M remains zero.
        self.M = np.array([
            [1, 0, 0, 0, 0],   # waist
            [0, 1, 0, 0, 0],   # shoulder
            [0, 0, 1, 0, 0],   # elbow
            [0, 0, 0, 0, 0],   # forearm_roll (unused)
            [0, 0, 0, 1, 0],   # wrist_angle
            [0, 0, 0, 0, 1],   # wrist_rotate
        ], dtype=float)

        # ----------------------
        # ROS Topics
        # ----------------------
        self.master_js_topic = "/rx150/joint_states"
        self.slave_js_topic  = "/vx300s/joint_states"
        self.master_cmd_topic = "/rx150/commands/joint_group"
        self.slave_cmd_topic  = "/vx300s/commands/joint_group"

        # ----------------------
        # State storage
        # ----------------------
        self.master_js = None
        self.slave_js = None
        self.master_sleep = None
        self.slave_sleep = None

        # ----------------------
        # Subscribers
        # ----------------------
        self.create_subscription(JointState, self.master_js_topic, self.master_cb, 10)
        self.create_subscription(JointState, self.slave_js_topic,  self.slave_cb,  10)

        # ----------------------
        # Publishers
        # ----------------------
        self.master_pub = self.create_publisher(JointGroupCommand, self.master_cmd_topic, 10)
        self.slave_pub  = self.create_publisher(JointGroupCommand, self.slave_cmd_topic,  10)

        # ----------------------
        # Timer Loop
        # ----------------------
        self.timer = self.create_timer(self.timer_dt, self.control_loop)

        # ----------------------
        # Wait for initial joint states
        # ----------------------
        self.get_logger().info("Waiting for joint states to sample sleep positions...")
        self.wait_for_initial_states()
        self.get_logger().info("Sleep positions sampled.")
        self.get_logger().info("Bilateral control started!")


    # ------------------------------------------------------------------
    # INITIAL STATE SAMPLING
    # ------------------------------------------------------------------
    def wait_for_initial_states(self):
        t0 = time.time()
        while time.time() - t0 < 5.0:
            if self.master_js and self.slave_js:
                mpos = np.array(self.master_js.position[:self.nM])
                spos = np.array(self.slave_js.position[:self.nS])

                self.master_sleep = mpos.copy()
                self.slave_sleep = spos.copy()
                return

            rclpy.spin_once(self, timeout_sec=0.1)


    # ------------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------------
    def master_cb(self, msg):
        self.master_js = msg

    def slave_cb(self, msg):
        self.slave_js = msg


    # ------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # ------------------------------------------------------------------
    def control_loop(self):
        if self.master_js is None or self.slave_js is None:
            return

        # ------------------------------
        # MASTER JOINT POSITIONS
        # ------------------------------
        mpos = np.array(self.master_js.position[:self.nM])
        dM   = mpos - self.master_sleep   # relative motion

        # ------------------------------
        # SLAVE TARGET
        # ------------------------------
        # dS = M * dM   → shape = (6,)
        slave_delta = self.M @ dM

        # scale movement
        slave_delta *= self.speed_scale

        slave_cmd = self.slave_sleep + slave_delta

        # Publish slave command
        slave_msg = JointGroupCommand()
        slave_msg.name = "arm"
        slave_msg.cmd = slave_cmd.tolist()
        self.slave_pub.publish(slave_msg)

        # ------------------------------
        # MASTER SOFT HOLD
        # ------------------------------
        # Prevent arm from falling when user lets go
        master_cmd = mpos * (1 - self.master_stiffness) \
                     + self.master_sleep * self.master_stiffness

        master_msg = JointGroupCommand()
        master_msg.name = "arm"
        master_msg.cmd = master_cmd.tolist()
        self.master_pub.publish(master_msg)

        # Done!


def main(args=None):
    rclpy.init(args=args)
    node = SimpleBilateral()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
