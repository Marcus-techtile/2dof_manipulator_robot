function dxdt = manipulator_dynamics_ode45(t, x, tau, noisy_force, use_potential_force, param)
    % ODE Function for manipulator dynamics
    % Inputs:
    %   t - Current time
    %   x - Current state vector [q; q_dot]
    %   tau - Torque input
    %   noisy_force - Disturbance force
    %   param - Robot parameters
    % Outputs:
    %   dxdt - State derivative vector [q_dot; q_ddot]
    
    % Extract states from the state vector
    q = x(1:2);
    q_dot = x(3:4);
    
    
    % Compute nominal model dynamics (without disturbance)
    [M, C, G, B] = manipulator_dynamics(q, q_dot, param.m1, param.m2, param.l1, param.l2, param.lc1, param.lc2, param.I1, param.I2, param.g, 0);
    
    if use_potential_force
    % Compute acceleration using the true model dynamics
        q_ddot = M \ (tau - C * q_dot - G + noisy_force);
    else
        q_ddot = M \ (tau - C * q_dot);
    end
    
    % Return the state derivatives
    dxdt = [q_dot; q_ddot];
end