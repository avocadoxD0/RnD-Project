#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
import numpy as np
import time

class HybridBilateral(Node):
    """
    Bilateral physical mode (master RX150, slave VX300s).
    - Master (rx150) is 5-DOF (manually moved or commanded)
    - Slave  (vx300s) is 6-DOF (follows master relative to its sleep pos)
    Mapping M is 6x5 so: slave_delta = M @ d_master
    Feedback from slave effort -> small position offset applied to master.
    """

    def __init__(self):
        super().__init__("hybrid_bilateral_node")

        # Topics / names (adjust if your namespaces differ)
        self.master_js_topic = "/rx150/joint_states"   # master (reactor) - 5 DOF
        self.slave_js_topic  = "/vx300s/joint_states"  # slave (viper) - 6 DOF
        self.master_cmd_topic = "/rx150/commands/joint_group"
        self.slave_cmd_topic  = "/vx300s/commands/joint_group"

        # Gains & safety params (tweak if needed)
        self.speed_scale = 1.6       # make slave faster (>1 speeds up)
        self.Kf = 0.018              # feedback gain from slave torque -> master offset (small)
        self.deadband = 0.006        # radians; ignore tiny jitter
        self.max_slave_move = 1.2    # rad clip for slave commands (absolute)
        self.max_master_feedback = 0.12  # rad clip for feedback applied to master
        self.tau_lp_alpha = 0.15     # low-pass alpha for torque signal
        self.offset_lp_alpha = 0.12  # low-pass alpha for master feedback offset
        self.timer_dt = 0.02         # 50 Hz

        # Mapping M (6 x 5) : slave_pos = slave_sleep + M @ (master - master_sleep)
        # Rows = slave DOFs (6), Cols = master DOFs (5)
        self.M = np.array([
            [1, 0, 0, 0, 0],   # slave waist    <- master[0]
            [0, 1, 0, 0, 0],   # slave shoulder <- master[1]
            [0, 0, 1, 0, 0],   # slave elbow    <- master[2]
            [0, 0, 0, 1, 0],   # slave forearm_roll <- master[3]
            [0, 0, 0, 0, 1],   # slave wrist_angle <- master[4]
            [0, 0, 0, 0, 0],   # slave wrist_rotate suppressed (extra DOF)
        ], dtype=float)

        # Storage
        self.master_js = None   # latest JointState for master
        self.slave_js = None    # latest JointState for slave
        self.master_sleep = None
        self.slave_sleep = None

        self.torque_lp = None   # filtered slave torque vector (length=5)
        self.offset_lp = np.zeros(6)  # filtered master feedback (6-long)

        # Subscribers
        self.create_subscription(JointState, self.master_js_topic, self.master_cb, 10)
        self.create_subscription(JointState, self.slave_js_topic, self.slave_cb, 10)

        # Publishers
        self.master_pub = self.create_publisher(JointGroupCommand, self.master_cmd_topic, 10)
        self.slave_pub  = self.create_publisher(JointGroupCommand, self.slave_cmd_topic, 10)

        # Timer control loop
        self.timer = self.create_timer(self.timer_dt, self.control_loop)

        self.get_logger().info("HybridBilateral node started — waiting for joint_states to sample sleep positions...")

        # safety: wait up to N seconds for initial joint state messages
        self._wait_for_initial_states(timeout_s=6.0)

    def _wait_for_initial_states(self, timeout_s=6.0):
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0 < timeout_s):
            if self.master_js is not None and self.slave_js is not None:
                try:
                    mpos = np.array(self.master_js.position)
                    spos = np.array(self.slave_js.position)
                except Exception:
                    break

                # ensure lengths fit expected dims; if not warn and continue
                if len(mpos) >= 5 and len(spos) >= 6:
                    self.master_sleep = mpos[:5].copy()    # master: 5
                    self.slave_sleep = spos[:6].copy()     # slave: 6
                    s_eff = np.array(self.slave_js.effort)[:5] if len(self.slave_js.effort) >= 5 else np.zeros(5)
                    self.torque_lp = s_eff.copy()
                    self.get_logger().info("Sampled sleep positions (master 5d, slave 6d). Ready.")
                    return
                else:
                    self.get_logger().warn(f"Unexpected joint_state lengths while sampling sleep: master={len(mpos)}, slave={len(spos)}")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn("Timed out waiting for joint_states — node will still run but sleep positions may be invalid.")

    def master_cb(self, msg: JointState):
        # store full message; we'll slice later
        self.master_js = msg

    def slave_cb(self, msg: JointState):
        self.slave_js = msg

    def control_loop(self):
        # must have both joint states and sleep positions
        if self.master_js is None or self.slave_js is None or self.master_sleep is None or self.slave_sleep is None:
            return

        # Read fresh positions (with safety checks)
        try:
            master_pos_raw = np.array(self.master_js.position)
            slave_pos_raw  = np.array(self.slave_js.position)
            slave_eff_raw  = np.array(self.slave_js.effort) if len(self.slave_js.effort) > 0 else np.zeros(len(slave_pos_raw))
        except Exception as e:
            self.get_logger().warn(f"joint state read error: {e}")
            return

        # verify sizes
        if master_pos_raw.size < 5 or slave_pos_raw.size < 6:
            self.get_logger().warn(f"Joint state sizes not as expected: master={master_pos_raw.size}, slave={slave_pos_raw.size}")
            return

        master_pos_raw = master_pos_raw[:5]   # master = 5
        slave_pos_raw  = slave_pos_raw[:6]    # slave  = 6
        slave_eff_raw  = slave_eff_raw[:5] if slave_eff_raw.size >= 5 else np.zeros(5)

        # --- compute master displacement from sleep (5,)
        d_master = master_pos_raw - self.master_sleep    # shape (5,)
        # deadband small noise
        d_master[np.abs(d_master) < self.deadband] = 0.0

        # --- slave target (relative to its sleep) ---
        # IMPORTANT: use M @ d_master because M is 6x5
        if self.M.shape != (6, 5):
            self.get_logger().error(f"Mapping M has unexpected shape {self.M.shape}; expected (6,5). Aborting this cycle.")
            return

        try:
            slave_delta = (self.M @ d_master) * self.speed_scale   # shape (6,)
        except Exception as e:
            self.get_logger().error(f"Matrix multiply error M @ d_master: {e}")
            return

        slave_target = self.slave_sleep + slave_delta

        # safety clip on slave target (absolute bounds)
        slave_target = np.clip(slave_target, -self.max_slave_move, self.max_slave_move)

        # publish slave command
        slave_msg = JointGroupCommand()
        slave_msg.name = "arm"
        slave_msg.cmd = [float(x) for x in slave_target.tolist()]
        self.slave_pub.publish(slave_msg)

        # --- process slave efforts: low-pass filter to avoid spikes ---
        if self.torque_lp is None:
            self.torque_lp = slave_eff_raw.copy()
        else:
            self.torque_lp = self.torque_lp * (1 - self.tau_lp_alpha) + slave_eff_raw * self.tau_lp_alpha

        # contact detection (for logging)
        if np.any(np.abs(self.torque_lp) > 0.6):
            self.get_logger().info("Slave contact/resistance detected (filtered effort high).")

        # --- compute small feedback to master from slave torque ---
        tau_s = self.torque_lp.copy()   # (5,)
        try:
            master_feedback = - self.Kf * (self.M @ tau_s)  # (6,)
        except Exception as e:
            self.get_logger().error(f"Matrix multiply error M @ tau_s: {e}")
            return

        # clip feedback to safe small magnitude (elementwise)
        master_feedback = np.clip(master_feedback, -self.max_master_feedback, self.max_master_feedback)

        # low-pass filter on offset for smoothing
        self.offset_lp = (1 - self.offset_lp_alpha) * self.offset_lp + self.offset_lp_alpha * master_feedback

        # apply tiny feedback only to master’s first 5 DOFs
        master_offset_to_apply = self.offset_lp[:5]
        master_offset_to_apply[np.abs(master_offset_to_apply) < 1e-4] = 0.0

        # Construct master command = current master positions + small offset
        master_cmd = master_pos_raw + master_offset_to_apply

        # Clip master target to small safe bounds around sleep
        master_cmd = np.clip(master_cmd, self.master_sleep - 0.9, self.master_sleep + 0.9)

        # Publish master command — small so it won't aggressively fight the human.
        master_msg = JointGroupCommand()
        master_msg.name = "arm"
        master_msg.cmd = [float(x) for x in master_cmd.tolist()]
        self.master_pub.publish(master_msg)

        # debug (throttled)
        if int(time.time() * 10) % 25 == 0:
            self.get_logger().debug(
                f"d_master={np.round(d_master,3).tolist()} | slave_delta={np.round(slave_delta,3).tolist()} | fb_master={np.round(master_offset_to_apply,3).tolist()}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = HybridBilateral()
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info("Shutting down hybrid bilateral node.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
