%% Simulation: 2-DOF Robot Model without Controller using ODE45
% This script simulates the 2-DOF robotic manipulator model without a controller
% using the ODE45 solver for improved accuracy.

% Add paths
current_dir = fileparts(mfilename('fullpath'));
addpath(current_dir);
addpath(fullfile(current_dir, 'robot_model'));  % Add path to robot model folder
addpath(fullfile(current_dir, '..'));
addpath(fullfile(current_dir, '..', '..', 'video'));

clear;
clc;

%% Setup Simulation Parameters
dt = 0.01;              % Time step for output/plotting
T_final = 15;           % Total simulation time
t = 0:dt:T_final;       % Time vector for output/plotting
n_steps = length(t);    % Number of steps for output/plotting

%% Manipulator Parameters (Nominal Values)
m1 = 5.0;                % Mass of link 1 (kg)
m2 = 4.5;                % Mass of link 2 (kg)
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
q_result = zeros(stateDim, n_steps);       % Joint positions [q1; q2]
q_dot_result = zeros(stateDim, n_steps);   % Joint velocities [q1_dot; q2_dot]
tau_result = zeros(stateDim, n_steps);     % Input torques [tau1; tau2]

% Set initial condition
q_result(:,1) = [0; 0];
q_dot_result(:,1) = [0; 0];

%% Manual Torque Input Definition
% Choose which torque input to use
use_constant_torque = false;  % Set to false to use time-varying torque

% Define the torque input for each joint
% Example 1: Constant torque
if use_constant_torque
    tau_constant = [1.0; 0.5];  % Constant torque vector
else
    % Example 2: Time-varying torque function 
    tau_function = @(t) [1.0 * sin(0.5 * t); 0.5 * cos(0.5 * t)];
end

%% External Disturbance Force
% Define the external disturbance (set to zero for this basic simulation)
noisy_force = [0; 0];  % No external disturbance

%% Set ODE solver options 
% Set ODE options including the output function for progress
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4*ones(4,1), 'MaxStep', 0.1);

%% Simulation Loop with ODE45
fprintf('Starting ODE45 simulation...\n');
sim_start_time = tic;

% Initial state vector [q1; q2; q1_dot; q2_dot]
initial_state = [q_result(:,1); q_dot_result(:,1)];

% Use a single ODE45 call for the entire simulation
use_potential_force = false;
if use_constant_torque
    % Use the constant torque if selected
    [t_ode, x_ode] = ode45(@(t_sim, x) manipulator_dynamics_ode45(t_sim, x, tau_constant, noisy_force, use_potential_force, param), t, initial_state, options);
else
    % Use a wrapper function for time-varying torque
    [t_ode, x_ode] = ode45(@(t_sim, x) manipulator_dynamics_ode45(t_sim, x, tau_function(t_sim), noisy_force, use_potential_force, param), t, initial_state, options);
end

% Extract and store results
for i = 1:length(t_ode)
    % Find the closest time point in our desired output time vector
    [~, idx] = min(abs(t - t_ode(i)));
    
    % Store results at the corresponding time point
    q_result(:,idx) = x_ode(i, 1:2)';
    q_dot_result(:,idx) = x_ode(i, 3:4)';
    
    % Store the applied torque
    if use_constant_torque
        tau_result(:,idx) = tau_constant;
    else
        tau_result(:,idx) = tau_function(t_ode(i));
    end
        % Display progress every 10% of steps
    if mod(i, floor(n_steps/10)) == 0
        sim_time =  toc(sim_start_time);
        fprintf('Progress: %.1f%% complete', 100*i/(n_steps-1));
        fprintf('. Elapsed time: %.2f seconds\n', sim_time);
    end
end

elapsed_time = toc(sim_start_time);
fprintf('\nSimulation complete! Total elapsed time: %.2f seconds\n\n', elapsed_time);

%% Plot Results
plot_2dof_model(t, q_result, q_dot_result, tau_result, l1, l2, true);
