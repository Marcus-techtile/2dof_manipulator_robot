%% Simulation: 2-DOF Robot Model without Controller
% This script simulates the 2-DOF robotic manipulator model without a controller.
% The input torques (tau) are set manually.

% Add paths
current_dir = fileparts(mfilename('fullpath'));
addpath(current_dir);
addpath(fullfile(current_dir, 'robot_model'));

clear;
clc;

%% Setup Simulation Parameters
dt = 0.01;              % Time step for simulation
T_final = 15;           % Total simulation time
t = 0:dt:T_final;       % Time vector
n_steps = length(t);    % Number of steps

%% Manipulator Parameters (Nominal Values)
m1 = 2.0;                % Mass of link 1 (kg)
m2 = 1.5;                % Mass of link 2 (kg)
l1 = 1.0;                % Length of link 1 (m)
l2 = 0.8;                % Length of link 2 (m)
lc1 = 0.5;               % Distance to center of mass of link 1 (m)
lc2 = 0.4;               % Distance to center of mass of link 2 (m)
I1 = (1/3) * m1 * l1^2;  % Inertia of link 1 (kg*m^2)
I2 = (1/3) * m2 * l2^2;  % Inertia of link 2 (kg*m^2)
g = 9.81;                % Gravity acceleration (m/s^2)

% Create parameter structure
param = struct('m1', m1, 'm2', m2, 'l1', l1, 'l2', l2, 'lc1', lc1, 'lc2', lc2, ...
    'I1', I1, 'I2', I2, 'g', g);

%% Initialize State Variables
stateDim = 2; 
q = zeros(stateDim, n_steps);       % Joint positions [q1; q2]
q_dot = zeros(stateDim, n_steps);   % Joint velocities [q1_dot; q2_dot]

% Set initial condition
q(:,1) = [0; 0];
q_dot(:,1) = [0; 0];

%% Manual Torque Input
% Define the torque input for each joint over time
% Example: constant torque
tau = [2; 0];  % Torque for joint 1 and joint 2

%% Main Simulation Loop
% Simulation options
use_potential_force = false;

for i = 1:n_steps-1
    % Current states
    q_i = q(:,i);
    q_dot_i = q_dot(:,i);
    
    % Compute nominal model dynamics
    [M, C, G, B] = manipulator_dynamics(q_i, q_dot_i, param.m1, param.m2, param.l1, param.l2, param.lc1, param.lc2, param.I1, param.I2, param.g, 0);
    
    % Compute acceleration
    if use_potential_force
        q_ddot = M \ (B*tau - C * q_dot_i - G);
    else
        q_ddot = M \ (B*tau - C * q_dot_i);
    end
    
    % Update states using Forward Euler integration
    q_dot(:,i+1) = q_dot_i + dt * q_ddot;
    q(:,i+1) = q_i + dt * q_dot_i;
end

%% Plot Results
figure('Position', [100, 100, 800, 600]);

%% Plot Results
plot_2dof_model(t, q, q_dot, tau, l1, l2, true);