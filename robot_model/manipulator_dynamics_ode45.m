function dxdt = manipulator_dynamics_ode45(t, x, t_ref, q_d_ref, q_d_dot_ref, q_d_ddot_ref, controller, param, dt_sim)
    % ODE Function for manipulator dynamics
    % Inputs:
    %   t - Current time
    %   x - Current state vector [q; q_dot]
    %   t_ref - Reference time vector
    %   q_d_ref - Reference joint positions
    %   q_d_dot_ref - Reference joint velocities
    %   q_d_ddot_ref - Reference joint accelerations
    %   controller - Controller object
    %   param - Robot parameters
    %   dt_sim - Simulation time step
    % Outputs:
    %   dxdt - State derivative vector [q_dot; q_ddot]
    
    % Extract states from the state vector
    q = x(1:2);
    q_dot = x(3:4);
    
    % Find the closest index in the reference time vector
    [~, idx] = min(abs(t_ref - t));
    
    % Get reference values at the closest time point
    q_d = q_d_ref(:,idx);
    q_d_dot = q_d_dot_ref(:,idx);
    q_d_ddot = q_d_ddot_ref(:,idx);
    
    % Compute nominal model dynamics (without disturbance)
    [M, C, G, B] = manipulator_dynamics(q, q_dot, param.m1, param.m2, param.l1, param.l2, param.lc1, param.lc2, param.I1, param.I2, param.g, 0);
    
    % Compute true model dynamics with uncertainties
    [M_true, C_true, G_true, B] = manipulator_dynamics(q, q_dot, param.m1, param.m2, param.l1, param.l2, param.lc1, param.lc2, param.I1, param.I2, param.g, noisy_force);
    
    % Compute control signal
    [tau, ~, ~, ~, ~] = controller.computeControl(q, q_dot, q_d, q_d_dot, q_d_ddot, M, C, G, B, dt_sim);
    
    % Compute acceleration using the true model dynamics
    q_ddot = M_true \ (tau - C_true * q_dot - G_true);
    
    % Return the state derivatives
    dxdt = [q_dot; q_ddot];
end