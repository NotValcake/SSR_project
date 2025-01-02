# SSR_project

This repository contains the solution for the Servo Systems and Robotics course project.

## Project Overview

The project focuses on the analysis and control of robotic manipulators, including tasks such as:

- **Kinematics**: Direct and inverse kinematics calculations (position, velocity, acceleration)
- **Dynamics**: Inverse dynamics computations
- **Trajectory Planning**: Implementation of cycloidal motion profiles and trajectory following
- **Visualization**: Plotting manipulator configurations and workspaces

## Repository Structure

The repository includes the following MATLAB scripts:

- `cycloidal.m`: Generates cycloidal trajectories
- `dirkin.m`: Computes direct kinematics
- `invdyn.m`: Calculates inverse dynamics
- `invkin.m`: Solves inverse kinematics
- `plotmanipulator.m`: Visualizes the manipulator in 3D space
- `plotsingularities.m`: Identifies and plots singular configurations

## TODO

The following tasks are still pending for this project:

1. **Kinematics**:
   - Complete velocity and acceleration computations for the direct and inverse kinematics.
   - Debug and optimize inverse kinematics for edge cases.

2. **Dynamics**:
   - Solution to inverse dynamics for the closed chain structure.
   - Add checks for singular configurations in dynamics computations.

3. **Control Design**:
   - Design a control structure for trajectory following (centralized or decentralized control).
   - Ensure continuity in position and velocity while respecting motor limits.

4. **Simulation**:
   - Simulate the trajectory task.
   - Verify continuity in motion parameters and adherence to motor constraints.

5. **Report Preparation**:
   - Document algorithms and notation used.
   - Provide simulation results for end-effector trajectory, joint motions, and motor torques.
   - Compare theoretical and simulated motions to evaluate control effectiveness.
   - Illustrate the debugging methodology for kinematics and dynamics with relevant graphs.

## Getting Started

To run the scripts:

1. **Clone the repository**:

   ```bash
   git clone https://github.com/NotValcake/SSR_project.git
   ```

2. **Navigate to the project directory**:

   ```bash
   cd SSR_project
   ```

3. **Open MATLAB** and add the project directory to the MATLAB path:

   ```matlab
   addpath('path_to_SSR_project')
   ```

4. **Execute the desired script**. For example, to run the direct kinematics computation:

   ```matlab
   testdirkin
   ```

## Prerequisites

- MATLAB R2020a or later
- Symbolic Math Toolbox
