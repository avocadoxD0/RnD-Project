# Arm-to-Arm Bilateral Control of Robotic Manipulators

## Overview
This R&D project presents the design, implementation, and validation of a **joint-space bilateral control framework** for robotic manipulators. The system enables real-time **motion synchronization and force reflection** between a master and a slave robotic arm, allowing one robot to perceive interaction forces experienced by the other.

The project uses the **Interbotix Reactor X150** as the master manipulator and the **Interbotix Viper X300** as the slave. The bilateral control loop was first validated in simulation using **ROS2, RViz, and PyBullet**, followed by partial deployment on physical hardware to study real-world limitations.

---

## Objectives
- Implement arm-to-arm bilateral control using joint-space mapping  
- Enable force reflection from the slave to the master during environmental interaction  
- Study transparency and stability characteristics of bilateral control  
- Compare simulation performance with real hardware behavior  

---

## System Architecture
The bilateral control system consists of two coupled information flows:

- **Forward Path (Master → Slave):**  
  Master joint angles are mapped to the slave using a predefined joint-space mapping matrix.

- **Feedback Path (Slave → Master):**  
  Reaction torques generated due to environment interaction are reflected back to the master.

A compliant **admittance control model** is implemented on the master side to convert reflected torques into smooth positional offsets when direct torque control is unavailable.

---

## Mathematical Framework
The control formulation follows standard rigid-body joint-space dynamics:

- Joint-space mapping between manipulators with mismatched DOF  
- Torque computation using Jacobian-transformed interaction forces  
- Gain-scaled force reflection to the master  
- Admittance-based compliance for stable force feedback  

Stability is addressed using passivity-based considerations, gain tuning, torque filtering, and dead-zone handling to prevent oscillations.

---

## Simulation & Validation
- Initial master–slave mapping verified in **RViz** using Interbotix URDF models  
- Full bilateral control loop implemented and tested in **PyBullet**  
- Simulation demonstrated:
  - Stable joint synchronization  
  - Correct force-feedback activation during contact  
  - Robust response under obstacle interaction  

Simulation results confirmed the effectiveness of joint-space bilateral control under idealized conditions.

---

## Hardware Deployment
The bilateral control framework was deployed on the physical **X150–X300** robotic arm pair. While partial bilateral motion was achieved, sustained synchronized operation was limited by:

- Control-loop timing inconsistencies  
- Python execution and communication latency  
- Actuator thermal and hardware constraints  
- Noise and uncertainty in torque estimation  

These observations highlight the gap between simulation and real-world implementation.

---

## Key Observations
- Joint-space bilateral control performs reliably in simulation  
- Force reflection behaves predictably during environment contact  
- Hardware deployment exposes challenges related to timing, actuation, and sensing  
- Real-world constraints significantly affect bilateral control stability  

---

## Tech Stack
- **Middleware:** ROS2  
- **Simulation:** PyBullet  
- **Visualization:** RViz  
- **Robotic Arms:** Interbotix Reactor X150, Interbotix Viper X300  
- **Actuators:** Dynamixel servo motors  
- **Programming Language:** Python  
- **Control Techniques:** Bilateral control, admittance control, joint-space mapping  

---

## Future Work
- Integration of full **IK/FK** for Cartesian-space bilateral control  
- Latency-aware and high-frequency control loops  
- Improved force sensing and torque estimation  
- Enhanced hardware reliability and thermal management  
- Extension to real-world teleoperation tasks  

---

## Author
**Farhaan Nasir**  
R&D Project – Robotics & Control Systems  

---

## License
This project is intended for academic and research purposes.
