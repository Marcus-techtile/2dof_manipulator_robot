function [M, C, G, B] = manipulator_dynamics(q, q_dot, m1, m2, l1, l2, lc1, lc2, I1, I2, g, noisy_force)
    % This function computes the dynamics matrices for a 2-DOF planar manipulator
    % Inputs:
    %   q     - Joint positions [q1; q2]
    %   q_dot - Joint velocities [q1_dot; q2_dot]
    %   m1, m2 - Masses of links (kg)
    %   l1, l2 - Lengths of links (m)
    %   lc1, lc2 - Distances to centers of mass (m)
    %   I1, I2 - Inertias of links (kg*m^2)
    %   g - Gravity acceleration (m/s^2)
    %   noisy_force - External disturbance force value to add to gravity vector
    % Outputs:
    %   M - Inertia matrix
    %   C - Coriolis and centrifugal matrix
    %   G - Gravity vector
    %   B - Input matrix
    
    % Extract joint positions and velocities
    q1 = q(1);
    q2 = q(2);
    q1_dot = q_dot(1);
    q2_dot = q_dot(2);
    
    % Precompute trigonometric terms
    s1 = sin(q1);
    c1 = cos(q1);
    s2 = sin(q2);
    c2 = cos(q2);
    s12 = sin(q1 + q2);
    c12 = cos(q1 + q2);
    
    % Compute inertia matrix M
    M = zeros(2, 2);
    M(1,1) = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2) + I1 + I2;
    M(1,2) = m2*(lc2^2 + l1*lc2*c2) + I2;
    M(2,1) = M(1,2);
    M(2,2) = m2*lc2^2 + I2;
    
    % Compute Coriolis matrix C
    h = -m2*l1*lc2*s2;
    C = zeros(2, 2);
    C(1,1) = -q2_dot*h;
    C(1,2) = -(q1_dot + q2_dot)*h;
    C(2,1) = q1_dot*h;
    C(2,2) = 0;
    
    % Compute gravity vector G
    G = zeros(2, 1);
    G(1) = (m1*lc1 + m2*l1)*g*c1 + m2*lc2*g*c12;
    G(2) = m2*lc2*g*c12;
    G = G + [noisy_force; noisy_force];

    % Input matrix
    B = [1 0; 0 1];
end

function noisy_force = calculate_disturbance(t, force_base)
    % Calculate disturbance force at a given time
    % Inputs:
    %   t - Current time
    %   force_base - Base force magnitude
    % Outputs:
    %   noisy_force - Calculated disturbance force
    
    if nargin < 2
        force_base = 20; % Default force base value
    end
    
    if (t < 2)  % Initial delay
        noisy_force = 0;
    elseif (t < 2.2)  % Ramp up to first force (0.2s)
        noisy_force = force_base * (t - 2)/0.2;  % Linear increase from 0 to force_base
    elseif (t < 4)
        noisy_force = force_base;
    elseif (t < 4.2)  % Ramp down to negative force (0.2s)
        noisy_force = force_base - 8 * (t - 4)/0.2;  % Linear decrease from force_base to -3
    elseif (t < 6)
        noisy_force = -3;
    elseif (t < 6.2)  % Ramp up to final force (0.2s)
        noisy_force = -3 + force_base * (t - 6)/0.2;  % Linear increase from -3 to 2
    else
        noisy_force = 2;
    end
end 