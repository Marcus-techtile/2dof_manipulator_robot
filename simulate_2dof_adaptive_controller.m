%% Simulation: 2-DOF Robot with Adaptive SMC Controller
% This script simulates the 2-DOF robotic manipulator with an adaptive sliding mode controller.

% Add paths
current_dir = fileparts(mfilename('fullpath'));
addpath(current_dir);
addpath(fullfile(current_dir, 'robot_model'));
addpath(fullfile(current_dir, 'controller'));
addpath(fullfile(current_dir, 'visualization'));
addpath(fullfile(current_dir, '..'));
addpath(fullfile(current_dir, '..', '..', 'video'));

clear;
clc;

%% Setup Simulation Parameters
dt = 0.01;              % Time step for simulation
T_final = 15;           % Total simulation time
t = 0:dt:T_final;       % Time vector
n_steps = length(t);    % Number of steps

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

%% Reference Trajectory: Sinusoidal path for both joints
omega1 = 0.2;                % Frequency for joint 1 (rad/s)
omega2 = 0.2;                % Frequency for joint 2 (rad/s)
amp1 = 0.5;                  % Amplitude for joint 1 (rad)
amp2 = 0.3;                  % Amplitude for joint 2 (rad)

q_d = zeros(2, n_steps);
q_d_dot = zeros(2, n_steps);
q_d_ddot = zeros(2, n_steps);

for i = 1:n_steps
    % Desired positions
    q_d_0 = [-0.1; 0.1];   % Initial position
    q_d(1,i) = q_d_0(1) + amp1 * sin(omega1 * t(i));
    q_d(2,i) = q_d_0(2) + amp2 * sin(omega2 * t(i));

    % Desired velocities
    q_d_dot(1,i) = amp1 * omega1 * cos(omega1 * t(i));
    q_d_dot(2,i) = amp2 * omega2 * cos(omega2 * t(i));
    
    % Desired accelerations
    q_d_ddot(1,i) = -amp1 * omega1^2 * sin(omega1 * t(i));
    q_d_ddot(2,i) = -amp2 * omega2^2 * sin(omega2 * t(i));
end

%% Initialize Adaptive SMC Controller
stateDim = 2;         % [q1; q2]
controlDim = 2;       % [tau1; tau2]

% Create controller with parameters
controller = AdaptiveSMCControl(stateDim, controlDim, ...
    'Lambda', 2, ...
    'SmcGain', 3, ...
    'Eta', 0.1, ...
    'Epsilon', 0.1, ...
    'GammaRho', 5, ...
    'Kappa', 0.1, ...
    'UseAdaptiveGain', true);

%% Initialize State Variables
q = zeros(stateDim, n_steps);       % Joint positions [q1; q2]
q_dot = zeros(stateDim, n_steps);   % Joint velocities [q1_dot; q2_dot]
tau = zeros(controlDim, n_steps);   % Control inputs [tau1; tau2]
tau_eq = zeros(controlDim, n_steps); % Equivalent control term
tau_rob = zeros(controlDim, n_steps); % Robust control term
s_norm = zeros(1, n_steps);         % Sliding variable norm
rho_hat = zeros(controlDim, n_steps); % Adaptive gain
delta_true = zeros(controlDim, n_steps); % True uncertainty/disturbance
e_all = zeros(stateDim, n_steps);   % Position errors
e_dot_all = zeros(stateDim, n_steps); % Velocity errors
s_all = zeros(stateDim, n_steps);   % Sliding variables
e_norm = zeros(1, n_steps);         % Norm of position errors

% Set initial conditions
q(:,1) = [0; 0];
q_dot(:,1) = [0; 0];

%% Set disturbance pattern
external_disturbance = zeros(controlDim, n_steps);
for i = 1:n_steps
    if t(i) > 3 && t(i) < 5
        % Constant disturbance
        external_disturbance(:,i) = [0; 2];
    end
end

%% Main Simulation Loop
fprintf('Starting simulation with Adaptive SMC Controller...\n');
tic;

for i = 1:n_steps-1
    % Current states
    q_i = q(:,i);
    q_dot_i = q_dot(:,i);
    
    % Calculate errors
    e = q_d(:,i) - q_i;
    e_dot = q_d_dot(:,i) - q_dot_i;
    
    % Store error values
    e_all(:,i) = e;
    e_dot_all(:,i) = e_dot;
    e_norm(i) = norm(e);
    
    % Compute model dynamics
    [M, C, G, B] = manipulator_dynamics(q_i, q_dot_i, param.m1, param.m2, param.l1, param.l2, param.lc1, param.lc2, param.I1, param.I2, param.g, 0);
    
    % Compute control signal using adaptive SMC
    [tau(:,i), s, tau_eq(:,i), tau_rob(:,i)] = controller.computeControl(q_i, q_dot_i, q_d(:,i), q_d_dot(:,i), q_d_ddot(:,i), M, C, G, B, dt);
    
    % Store sliding variables and other values
    s_all(:,i) = s;
    s_norm(i) = norm(s);
    rho_hat(:,i) = controller.rho_hat;
    
    % Get current disturbance
    disturbance = external_disturbance(:,i);
    delta_true(:,i) = disturbance;  % Store as true uncertainty
    
    % Compute acceleration with the external disturbance
    q_ddot = M \ (B * tau(:,i) - C * q_dot_i - G + disturbance);
    
    % Update states using Forward Euler integration
    q_dot(:,i+1) = q_dot_i + dt * q_ddot;
    q(:,i+1) = q_i + dt * q_dot_i;
    
    % Display progress every 10% of steps
    if mod(i, floor(n_steps/10)) == 0
        elapsed = toc;
        fprintf('Progress: %3d%% complete (Elapsed: %.1f s)\n', ...
                floor(100*i/(n_steps-1)), elapsed);
    end
end

elapsed_time = toc;
fprintf('Simulation complete! Total elapsed time: %.2f seconds\n\n', elapsed_time);

%% Plot Results using plot_2dof_results function
% Define which plots to show
plotOptions = struct(...
    'tracking', true, ...      % Show tracking performance plots
    'errors', false, ...        % Show error and sliding variable plots
    'control', false, ...       % Show control component plots
    'disturbance', false, ...   % Show disturbance and compensation plots
    'adaptiveGain', true, ...  % Show adaptive gain plot
    'animation', true, ...     % Show animation
    'saveVideo', false);       % Don't save video

% Call the plotting function
plot_2dof_results(t, q, q_d, tau, tau_eq, tau_rob, delta_true, e_all, e_dot_all, s_all, e_norm, s_norm, rho_hat, true, l1, l2, plotOptions);

