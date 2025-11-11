import pybullet as p
import pybullet_data
import time
import os
import numpy as np

# === Setup ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# === Load Environment ===
p.loadURDF("plane.urdf")

# --- Add a wall (static box) ---
wall_visual = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.02, 1.0, 1.0],   # (thickness, width, height)
    rgbaColor=[1, 0, 0, 1]
)

wall_collision = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.02, 0.3, 0.3]
)

# Rotate the wall by 90 degrees around the Z-axis
orientation = p.getQuaternionFromEuler([0, 0, 1.57])  # (roll, pitch, yaw)

wall = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=wall_collision,
    baseVisualShapeIndex=wall_visual,
    basePosition=[0.8, 0.4, 0.3],   # <- adjust as needed
    baseOrientation=orientation
)



# === Load URDF Files ===
urdf_files_path = os.path.expanduser("~/pybullet_mirror/urdf/interbotix_xsarm_descriptions/urdf")
rx150_path = os.path.join(urdf_files_path, "rx150.urdf")
vx300_path = os.path.join(urdf_files_path, "vx300.urdf")

mesh_search_path = os.path.expanduser("~/pybullet_mirror/urdf")
p.setAdditionalSearchPath(mesh_search_path)

print(f"Loading Master (RX150) from: {rx150_path}")
rx150 = p.loadURDF(rx150_path, [1.5, -0.5, 0], useFixedBase=True)

print(f"Loading Slave (VX300) from: {vx300_path}")
vx300 = p.loadURDF(vx300_path, [0.5, 0, 0], useFixedBase=True)

# === Joint Mapping Utilities ===
def get_movable_joints(robot_id):
    joints = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        if info[2] != p.JOINT_FIXED:
            joints.append((info[0], info[1].decode('utf-8')))
    return joints

master_joints = get_movable_joints(rx150)
slave_joints_map = {name: idx for idx, name in get_movable_joints(vx300)}
master_joints_map = {name: idx for idx, name in master_joints}

print("-" * 20)
print(f"Master Movable Joints: {[name for _, name in master_joints]}")
print(f"Slave Joint Map: {slave_joints_map}")
print("-" * 20)

# === GUI Sliders ===
slider_ids = []
for idx, name in master_joints:
    joint_info = p.getJointInfo(rx150, idx)
    low, high = joint_info[8], joint_info[9]
    slider_ids.append(p.addUserDebugParameter(name, low, high, 0))



# === Bilateral Control Parameters ===
M = np.eye(len(master_joints))   # 1-to-1 mapping
B = 8.0      # Damping
K = 15.0     # Stiffness
Kf = 0.02    # Force feedback gain scaling
dt = 1/240   # Simulation step
theta_offset = np.zeros(len(master_joints))





# === Main Loop ===
loop = 0
while True:
    # --- Read user input (Master Command) ---
    master_cmd = np.array([p.readUserDebugParameter(s) for s in slider_ids])
    slave_cmd = M @ master_cmd

    # --- Force Feedback Computation ---
    slave_torques = []
    for joint_name, master_idx in master_joints_map.items():
        slave_idx = slave_joints_map[joint_name]
        torque = p.getJointState(vx300, slave_idx)[3]
        slave_torques.append(torque)
    slave_torques = np.array(slave_torques)

    # τ_m = -Kf * Mᵀ * τ_s
    tau_m = -Kf * (M.T @ slave_torques)

    # === Safety Filters ===
    # Deadband (ignore small torques)
    tau_m[np.abs(tau_m) < 0.02] = 0.0

    # Torque saturation
    tau_m = np.clip(tau_m, -0.4, 0.4)

    # Admittance dynamics update
    theta_offset = theta_offset - dt * (K * theta_offset + tau_m) / B

    # Offset limit to prevent collapsing
    theta_offset = np.clip(theta_offset, -0.25, 0.25)

    # Corrected master command
    master_cmd_corrected = master_cmd + theta_offset

    # --- Apply Commands to Both Arms ---
    for i, (master_idx, joint_name) in enumerate(master_joints):
        # MASTER updates with feedback correction
        p.setJointMotorControl2(rx150, master_idx, p.POSITION_CONTROL,
                                targetPosition=float(master_cmd_corrected[i]))

        # SLAVE mirrors motion directly
        slave_idx = slave_joints_map[joint_name]
        p.setJointMotorControl2(vx300, slave_idx, p.POSITION_CONTROL,
                                targetPosition=float(slave_cmd[i]))

    # --- Debug Print Joint Data (every few seconds) ---
    if loop % 240 == 0:
        print("\n=== JOINT DATA ===")
        for joint_name, master_idx in master_joints_map.items():
            m = p.getJointState(rx150, master_idx)
            s = p.getJointState(vx300, slave_joints_map[joint_name])
            print(f"{joint_name}: MASTER({m[0]:.2f}, {m[1]:.2f}, {m[3]:.2f}) | "
                  f"SLAVE({s[0]:.2f}, {s[1]:.2f}, {s[3]:.2f})")

        # --- Debug Print Joint Data (every few seconds) ---
    if loop % 240 == 0:
        print("\n=== JOINT DATA ===")
        for joint_name, master_idx in master_joints_map.items():
            m = p.getJointState(rx150, master_idx)
            s = p.getJointState(vx300, slave_joints_map[joint_name])
            print(f"{joint_name}: MASTER({m[0]:.2f}, {m[1]:.2f}, {m[3]:.2f}) | "
                  f"SLAVE({s[0]:.2f}, {s[1]:.2f}, {s[3]:.2f})")

    # --- Contact Detection ---
    contacts = p.getContactPoints(bodyA=vx300, bodyB=wall)
    if len(contacts) > 0:
        print("Contact detected: Slave arm is touching the wall!")

    loop += 1
    p.stepSimulation()
    time.sleep(dt)