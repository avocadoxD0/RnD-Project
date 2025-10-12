import pybullet as p
import pybullet_data
import time
import os

# === Setup ===
# Connect to the physics server
p.connect(p.GUI)

# Add the default PyBullet assets path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set simulation parameters
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0) # We will step the simulation manually

# === Load Environment ===
p.loadURDF("plane.urdf")

# === 1. DEFINE CORRECT PATHS ===
# The URDF files are inside the description package you copied and converted.
# This path points to the folder containing the final .urdf files.
urdf_files_path = os.path.expanduser("~/pybullet_mirror/urdf/interbotix_xsarm_descriptions/urdf")
rx150_path = os.path.join(urdf_files_path, "rx150.urdf")
vx300_path = os.path.join(urdf_files_path, "vx300.urdf")

# This path points to the directory that CONTAINS the 'interbotix_xsarm_descriptions' folder.
# This allows PyBullet to find the meshes referenced inside the URDF.
mesh_search_path = os.path.expanduser("~/pybullet_mirror/urdf")
p.setAdditionalSearchPath(mesh_search_path)

# === Load Both Arms ===
print(f"Loading Master (RX150) from: {rx150_path}")
rx150 = p.loadURDF(rx150_path, [-0.5, 0, 0], useFixedBase=True)

print(f"Loading Slave (VX300) from: {vx300_path}")
vx300 = p.loadURDF(vx300_path, [0.5, 0, 0], useFixedBase=True)

# === 2. ROBUST JOINT MAPPING BY NAME ===
def get_movable_joints(robot_id):
    """Gets a list of (joint_index, joint_name) for movable joints, excluding the gripper."""
    joints = []
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode('utf-8')
        # We only want revolute/prismatic joints for the arm itself
        if info[2] != p.JOINT_FIXED:
             joints.append((info[0], joint_name))
    return joints

# Get the master arm's joints to create sliders for
master_joints = get_movable_joints(rx150)

# Create a dictionary to map slave joint names to their indices for quick lookups
slave_joints_map = {name: idx for idx, name in get_movable_joints(vx300)}

# Create a map for master joints as well for easy data access <--- ADDED
master_joints_map = {name: idx for idx, name in master_joints}

print("-" * 20)
print(f"Master Movable Joints: {[name for _, name in master_joints]}")
print(f"Slave Joint Map: {slave_joints_map}")
print("-" * 20)


# === Create GUI Sliders for Master Arm Control ===
slider_ids = []
for idx, name in master_joints:
    # Get the joint limits to make sliders more accurate
    joint_info = p.getJointInfo(rx150, idx)
    low_limit = joint_info[8]
    high_limit = joint_info[9]
    slider = p.addUserDebugParameter(f"master_{name}", low_limit, high_limit, 0)
    slider_ids.append(slider)

# === Main Simulation Loop ===
loop_count = 0 # <--- ADDED: Counter for periodic printing

while True:
    # Read the target positions from the GUI sliders
    target_positions = [p.readUserDebugParameter(s) for s in slider_ids]

    # Loop through all the master joints to control both arms
    for i, (master_joint_index, master_joint_name) in enumerate(master_joints):
        target_pos = target_positions[i]
        
        # 1. Control the Master Arm (RX150)
        p.setJointMotorControl2(
            bodyUniqueId=rx150,
            jointIndex=master_joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_pos
        )
        
        # 2. Mirror to the Slave Arm (VX300)
        # Check if the slave arm has a joint with the same name
        if master_joint_name in slave_joints_map:
            slave_joint_index = slave_joints_map[master_joint_name]
            p.setJointMotorControl2(
                bodyUniqueId=vx300,
                jointIndex=slave_joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_pos
            )

    # ==========================================================
    # === ADDED: GET AND PRINT JOINT DATA FOR BOTH ARMS ===
    # ==========================================================
    if loop_count % 60 == 0: # Print data periodically
        print("\n" + "="*40)
        print(f"JOINT STATES AT STEP {loop_count}")
        print("="*40)

        # Iterate through all movable joints of the master arm
        for joint_name, master_idx in master_joints_map.items():
            
            # Get and print Master joint data
            master_state = p.getJointState(rx150, master_idx)
            print(f"--- MASTER Joint: '{joint_name}' ---")
            print(f"  Pos: {master_state[0]:.3f} rad | Vel: {master_state[1]:.3f} rad/s | Torque: {master_state[3]:.3f} Nm")

            # Get and print corresponding Slave joint data
            if joint_name in slave_joints_map:
                slave_idx = slave_joints_map[joint_name]
                slave_state = p.getJointState(vx300, slave_idx)
                print(f"--- SLAVE  Joint: '{joint_name}' ---")
                print(f"  Pos: {slave_state[0]:.3f} rad | Vel: {slave_state[1]:.3f} rad/s | Torque: {slave_state[3]:.3f} Nm")

    loop_count += 1
    # ==========================================================

    # Advance the simulation by one step
    p.stepSimulation()
    # Wait a small amount of time to run at a realistic speed
    time.sleep(1. / 240.)