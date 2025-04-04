# 2-DOF Manipulator Simulation

This project demonstrates the implementation of a 2-DOF robotic manipulator. The simulation is implemented using MATLAB and includes both direct simulation and ODE45-based simulation approaches.

## Files

- `simulate_2dof_robot.m`: Simulates the 2-DOF robot model without controller using direct numerical integration.
- `simulate_2dof_robot_ode45.m`: Simulates the 2-DOF robot model without controller using the ODE45 solver for improved accuracy.
- `robot_model/manipulator_dynamics.m`: Contains the robot model functions, including the dynamics calculations, separated from the main simulation files for modularity and reusability.
- `robot_model/manipulator_dynamics_ode45.m`: ODE function for the robot model with ODE45 solver integration.
- `plot_2dof_model.m`: Plotting and animation functions for the 2-DOF manipulator model.

## Usage

1. Ensure MATLAB is installed on your system.
2. Run the desired script (`simulate_2dof_robot.m` or `simulate_2dof_robot_ode45.m`) to start the simulation.
3. The simulation will display plots of the joint positions, velocities, input torques, and an animation of the manipulator.

## Features

- Direct numerical integration and ODE45 solver options for simulation.
- Manual control inputs with options for constant or time-varying torques.
- Modular design with separated robot model functions for better code organization.
- Visualization of joint states, input torques, end-effector trajectory, and robot animation.
- Progress monitoring during ODE45 simulation.

## Requirements

- MATLAB R2020a or later.

## Project Structure

```
2dof_manipulator/
│
├── simulate_2dof_robot.m         # Direct integration simulation
├── simulate_2dof_robot_ode45.m   # ODE45-based simulation
├── plot_2dof_model.m             # Plotting and animation functions
│
└── robot_model/
    ├── manipulator_dynamics.m       # Core dynamics functions
    └── manipulator_dynamics_ode45.m # ODE45 compatible dynamics