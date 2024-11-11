# Baxter Robot Arm Visual Servoing Control

This repository contains code and resources for task-based control of the Baxter robot arm, developed as part of an advanced robotics lab. The goal is to implement visual servoing to control Baxter’s arm, first in a simulated environment (using CoppeliaSim) and then on a real Baxter robot.

## Project Overview

In this lab, we control Baxter’s arm to center a colored sphere in its camera view at a defined distance. This involves computing joint velocities to perform classical visual servoing and handling joint constraints to avoid unnecessary rotations and joint limits.

Key tools:
- **CoppeliaSim**: For Baxter robot simulation.
- **ROS (Robot Operating System)**: For managing communication between the control program and the simulator.
- **ViSP (Visual Servoing Platform)**: For matrix manipulations and other linear algebra tasks.

## Environment Setup

1. **ROS Setup**: Ensure ROS 1 is set up. Launch a terminal and type:
   ```bash
   ros1ws
2. **Clone repository**
3. **Build the package** using the following command in the ros directory:
   ```bash
   catkin build
4. **Run the sim environment**
   ```bash
   roslaunch ecn_baxter_vs sim.launch

## Main features  
  1. Robot Initialization:
        - The BaxterArm object initializes the right arm of the Baxter robot, connecting to its control interface. It retrieves the robot's initial joint positions, as well as its joint limits `(qmin, qmax)`.
        - Joint limits are processed to determine safe bounds `(qsm, qsp)` that provide a margin to avoid joint limits.

  2. Visual Features Setup:
        - A 2D feature point (p) and its desired position (pd) are defined. pd sets the target position for a green sphere to be centered at the origin with a depth of 1 meter.
        - The program tracks the image plane error (e), representing the difference between the current and desired features, including position, area, and camera orientation. A larger error vector (et) incorporates both visual and joint position errors (the latter is used to ensure that the joints stay away from the joint limits keeping them as best as possible near their middle value).

  3. Control Loop:
        - The main control loop runs continuously while the arm is active, performing the following tasks:
            - Feature Update: It fetches the current position, size (area), and orientation of the target sphere in the camera view. The error vector e is updated based on these values.
            - Interaction Matrix (L): The interaction matrix L maps feature errors to the camera twist.
            - Weighting Matrix (H): A diagonal matrix H is built to manage joint limit constraints using safety margins (qsp and qsm), ensuring joints stay within safe bounds.
            - Extended Jacobian: The code combines L with the robot’s camera Jacobian to be placed inside an extended Jacobian Jt that includes both visual and joint limit constraints. This Jacobian allows the robot to handle multiple constrains seamlessly.

  4. Joint Velocity Calculation:
        - The weighted pseudoinverse of the Jacobian `(HJt.pseudoInverse())` is used to compute joint velocities `(qdot)` based on the weighted error vector `(et)`. The gain (lambda) helps control the convergence speed of the visual servoing.

  5. Command Execution and Visualization:
        - The computed joint velocities are sent to the Baxter robot to adjust its arm’s position and orientation.
        - Finally, the program visualizes the current error values to monitor the progress of the visual servoing task.

