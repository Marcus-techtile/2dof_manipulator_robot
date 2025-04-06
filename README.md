# 2-DOF Manipulator Simulation

This project demonstrates the implementation of a 2-DOF robotic manipulator with various control and simulation approaches. The simulation is implemented using MATLAB and includes direct integration, ODE45-based simulation, and adaptive sliding mode control.

## Files

### Simulation Scripts
- `simulate_2dof_robot.m`: Simulates the 2-DOF robot model without controller using direct numerical integration.
- `simulate_2dof_robot_ode45.m`: Simulates the 2-DOF robot model without controller using the ODE45 solver for improved accuracy.
- `simulate_2dof_adaptive_controller.m`: Simulates the 2-DOF robot with an adaptive sliding mode controller for robust performance under uncertainty.

### Robot Model
- `robot_model/manipulator_dynamics.m`: Contains the robot model functions, including the dynamics calculations, separated from the main simulation files for modularity and reusability.
- `robot_model/manipulator_dynamics_ode45.m`: ODE function for the robot model with ODE45 solver integration.

### Controllers
- `controller/AdaptiveSMCControl.p`: Protected implementation of the adaptive sliding mode controller. Uses adaptive gain to handle uncertainties and disturbances.

### Visualization
- `visualization/plot_2dof_model.m`: Basic plotting and animation functions for the 2-DOF manipulator model without controller.
- `visualization/plot_2dof_results.m`: Enhanced plotting functions for visualization of controller performance, including tracking, errors, control signals, disturbance compensation, and adaptive gains.

## Usage

1. Ensure MATLAB is installed on your system (R2020a or later).
2. Run one of the simulation scripts:
   - `simulate_2dof_robot.m` for basic simulation without controller
   - `simulate_2dof_robot_ode45.m` for higher accuracy simulation without controller
   - `simulate_2dof_adaptive_controller.m` for simulation with adaptive sliding mode control
3. The simulation will display plots of joint positions, velocities, control signals, tracking errors, and an animation of the manipulator.

## Features

- Multiple simulation approaches:
  - Direct numerical integration 
  - ODE45 solver for improved accuracy
  - Adaptive sliding mode control for robust performance
- Adaptive controller with:
  - Sliding mode control for robustness
  - Adaptive gain for handling uncertainties
  - Boundary layer to reduce chattering
- Comprehensive visualization tools:
  - Joint position and velocity plots
  - Tracking error visualization
  - Control signal decomposition
  - End-effector trajectory display
  - Interactive animation with actual and desired position tracking
- Modular design with separated model, controller, and visualization components
- Progress monitoring during simulation

## Requirements

- MATLAB R2020a or later.

## Project Structure

```
2dof_manipulator/
│
├── simulate_2dof_robot.m             # Basic simulation with direct integration
├── simulate_2dof_robot_ode45.m       # ODE45-based simulation
├── simulate_2dof_adaptive_controller.m # Simulation with adaptive SMC
│
├── robot_model/
│   ├── manipulator_dynamics.m        # Core dynamics functions
│   └── manipulator_dynamics_ode45.m  # ODE45 compatible dynamics
│
├── controller/
│   └── AdaptiveSMCControl.p          # Adaptive SMC controller
│
└── visualization/
    ├── plot_2dof_model.m             # Basic plotting functions
    └── plot_2dof_results.m           # Enhanced visualization for controllers