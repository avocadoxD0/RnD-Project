# Bilateral Control of Robotic Arms

## Overview
This R&D project focuses on the design and implementation of a **bilateral control framework** for robotic manipulators, enabling stable **force–position feedback** for teleoperation tasks. The system allows an operator (master arm) to interact with a remote environment through a slave arm while receiving realistic force feedback, ensuring transparency and stability.

The project emphasizes control-theoretic formulation, simulation-based validation, and modern robotics middleware integration.

---

## Objectives
- Implement bilateral control for robotic arm teleoperation  
- Achieve stable force and position feedback between master and slave  
- Analyze transparency and stability under varying interaction forces  
- Validate control performance in a physics-based simulation environment  

---

## System Architecture
- **Master–Slave Manipulator Model**
- Bilateral feedback loop for position and force exchange
- Controller layer implementing PID / impedance-based control
- Simulation and visualization layer using ROS2 tools

---

## Control Strategy
- Dynamic modeling of robotic manipulators
- Position–force bilateral control scheme
- PID and impedance-based controllers for stability and transparency
- Performance evaluation using response analysis and disturbance rejection

---

## Tech Stack
- **Middleware:** ROS2  
- **Simulation:** PyBullet  
- **Visualization:** RViz  
- **Programming Language:** Python  
- **Control:** PID / Impedance Control  
- **Robotics Concepts:** Teleoperation, Force Feedback, Bilateral Control  

---

## Simulation & Validation
- Physics-based simulation of robotic arms in PyBullet  
- Real-time state visualization in RViz  
- Evaluation of system response, force tracking, and stability

---

## Key Learnings
- Practical implementation of bilateral control in robotic systems  
- Integration of control theory with modern robotics frameworks  
- Stability and transparency trade-offs in teleoperation systems  

---

## Future Work
- Hardware implementation with real robotic manipulators  
- Latency-aware and delay-compensated bilateral control  
- Advanced control schemes (passivity-based / adaptive control)  

---

## Author
**Farhaan**  
Robotics & Control Systems Enthusiast  

---

## License
This project is intended for academic and research purposes.
