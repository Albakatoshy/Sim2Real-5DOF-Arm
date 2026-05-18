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

* **`CAD_SolidWorks/`**
    Contains the original SolidWorks part (`.SLDPRT`) and assembly (`.SLDASM`) files, along with the exported `.STL` meshes used to 3D print the physical chassis of the robotic arm.

* **`MATLAB_Simscape/`**
    Holds the core Simulink (`.slx`) models for computing forward and inverse kinematics, along with Jacobian matrices and trajectory planning. Includes the initialization data scripts (`.m`), the multibody assembly definition (`.xml`), and all required `.STEP` geometry files for accurate 3D rendering.

* **`ROS2_Workspace/`**
    Contains the `robot_arm_uni` ROS 2 package. This includes the exported URDF files, MoveIt 2 configuration packages, and the launch files required to simulate the robot in Gazebo/RViz and communicate with the physical hardware via `ros2_control`.

* **`Arduino_Firmware/`**
    Includes the communication scripts (such as `arduino_base.m` and `send_esp.m`) that act as the bridge between the high-level software and the physical world, transmitting real-time joint angles over serial to actuate the motors.

* **`Hardware_PCB/`**
    Contains the schematics and design files for the custom printed circuit board (PCB) engineered specifically for this project to eliminate messy wiring and ensure reliable signal transmission to the servos.

* **`Media/`**
    A gallery of project documentation, including screenshots of the block diagrams, Gazebo simulation visuals, and video demonstrations of the physical Sim2Real synchronization in action.
