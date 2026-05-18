# 5-DOF Robotic Arm: Simulation to Reality 🤖

This repository contains the complete workflow for a custom 5-degree of freedom robotic arm, moving from 3D design to full software simulation and physical hardware execution. 

**Alexandria University | 4th Year Engineering Project**
**Team IDs:** 8787, 8774, 8803

## 🛠️ Tech Stack & Workflow
*   **Mechanical Design:** SolidWorks (Parts & Assemblies)
*   **Fabrication:** 3D Printing
*   **Simulation & Control:** ROS 2, Gazebo, RViz, MoveIt
*   **Kinematics & Dynamics:** MATLAB Simscape
*   **Hardware Bridge:** `ros2_control`, Arduino, Custom PCB

## 🚀 Project Phases
1.  **CAD Design:** Modeled from scratch in SolidWorks and exported to URDF.
2.  **Simulation:** Verified joint limits and trajectories in ROS 2 (Gazebo/RViz) and MATLAB.
3.  **Fabrication:** 3D printed the chassis and designed a custom PCB to eliminate wiring issues.
4.  **Hardware Integration:** Serial communication between MoveIt, MATLAB, and the physical robot using `ros2_control` ensuring exact positional matching.

## 📂 Repository Structure
*Briefly explain what is in each folder here.*
