function plot_2dof_model(t, q, q_dot, tau, l1, l2, animate)
    % Plot results for 2DOF manipulator model simulation without controller
    % Inputs:
    %   t - Time vector
    %   q - Joint positions
    %   q_dot - Joint velocities
    %   tau - Input torques
    %   l1, l2 - Link lengths
    %   animate - Whether to animate the manipulator

    % Default animation to true if not provided
    if nargin < 7
        animate = true;
    end
    
    %% Plotting Joint Positions and Velocities
    figure('Name', 'Joint States', 'Position', [100, 100, 800, 600]);

    % Joint Position Plotting
    subplot(2,2,1);
    plot(t, q(1,:), 'b-', 'LineWidth', 2);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Position (rad)', 'FontSize', 12);
    title('Joint 1 Position', 'FontSize', 14);
    grid on;

    subplot(2,2,2);
    plot(t, q(2,:), 'r-', 'LineWidth', 2);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Position (rad)', 'FontSize', 12);
    title('Joint 2 Position', 'FontSize', 14);
    grid on;

    % Joint Velocity Plotting
    subplot(2,2,3);
    plot(t, q_dot(1,:), 'b-', 'LineWidth', 2);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Velocity (rad/s)', 'FontSize', 12);
    title('Joint 1 Velocity', 'FontSize', 14);
    grid on;

    subplot(2,2,4);
    plot(t, q_dot(2,:), 'r-', 'LineWidth', 2);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Velocity (rad/s)', 'FontSize', 12);
    title('Joint 2 Velocity', 'FontSize', 14);
    grid on;

    %% Input Torques
    figure('Name', 'Input Torques', 'Position', [150, 150, 800, 400]);

    % Input torque for joint 1
    subplot(1,2,1);
    plot(t, tau(1,:), 'b-', 'LineWidth', 2);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Torque (N·m)', 'FontSize', 12);
    title('Input Torque - Joint 1', 'FontSize', 14);
    grid on;

    % Input torque for joint 2
    subplot(1,2,2);
    plot(t, tau(2,:), 'r-', 'LineWidth', 2);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Torque (N·m)', 'FontSize', 12);
    title('Input Torque - Joint 2', 'FontSize', 14);
    grid on;

    %% End-Effector Trajectory
    figure('Name', 'End-Effector Trajectory', 'Position', [200, 200, 600, 500]);
    
    % Calculate end-effector position
    x_ee = zeros(1, length(t));
    y_ee = zeros(1, length(t));
    
    for i = 1:length(t)
        % Forward kinematics
        x_ee(i) = l1*cos(q(1,i)) + l2*cos(q(1,i) + q(2,i));
        y_ee(i) = l1*sin(q(1,i)) + l2*sin(q(1,i) + q(2,i));
    end
    
    % Plot end-effector trajectory
    plot(x_ee, y_ee, 'g-', 'LineWidth', 2);
    xlabel('X Position (m)', 'FontSize', 12);
    ylabel('Y Position (m)', 'FontSize', 12);
    title('End-Effector Trajectory', 'FontSize', 14);
    axis equal;
    grid on;

    %% Animate the 2DOF Manipulator
    if animate
        animate_2dof_model(t, q, l1, l2);
    end
end

function animate_2dof_model(t, q, l1, l2)
    % Animate the 2DOF manipulator model
    % Inputs:
    %   t - Time vector
    %   q - Joint positions
    %   l1, l2 - Link lengths
    
    figure('Name', 'Manipulator Animation', 'Position', [100, 100, 800, 600]);
    
    % Set axis limits with some margin
    axis_limit = (l1 + l2) * 1.1;
    ax = axes;
    hold(ax, 'on');
    xlim([-axis_limit, axis_limit]);
    ylim([-axis_limit, axis_limit]);
    axis equal;
    grid on;
    title('2DOF Manipulator Animation', 'FontSize', 14);
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    
    % Create graphics objects for the links and joints
    link1_line = line('XData', [0, 0], 'YData', [0, 0], 'Color', 'blue', 'LineWidth', 3);
    link2_line = line('XData', [0, 0], 'YData', [0, 0], 'Color', 'red', 'LineWidth', 3);
    joint0_marker = plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    joint1_marker = plot(0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    ee_marker = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Create trace arrays to store the paths
    x_trace = [];
    y_trace = [];
    trace_line = plot(0, 0, 'g-', 'LineWidth', 1.5);
    
    % Create trace arrays for first joint
    x_trace_joint1 = [];
    y_trace_joint1 = [];
    trace_line_joint1 = plot(0, 0, 'c-', 'LineWidth', 1.5);
    
    % Animation speed control
    skip_frames = 10;
    
    legend('Link 1', 'Link 2', 'Base', 'Joint 1', 'End Effector', 'End-Effector Path', 'Joint 1 Path', 'FontSize', 10);
    
    % Animation loop
    for i = 1:skip_frames:length(t)
        % Calculate joint positions using forward kinematics
        x1 = l1*cos(q(1,i));
        y1 = l1*sin(q(1,i));
        x2 = x1 + l2*cos(q(1,i) + q(2,i));
        y2 = y1 + l2*sin(q(1,i) + q(2,i));
        
        % Update link positions
        set(link1_line, 'XData', [0, x1], 'YData', [0, y1]);
        set(link2_line, 'XData', [x1, x2], 'YData', [y1, y2]);
        
        % Update joint and end-effector positions
        set(joint1_marker, 'XData', x1, 'YData', y1);
        set(ee_marker, 'XData', x2, 'YData', y2);
        
        % Update end-effector trace
        x_trace = [x_trace, x2];
        y_trace = [y_trace, y2];
        set(trace_line, 'XData', x_trace, 'YData', y_trace);
        
        % Update first joint trace
        x_trace_joint1 = [x_trace_joint1, x1];
        y_trace_joint1 = [y_trace_joint1, y1];
        set(trace_line_joint1, 'XData', x_trace_joint1, 'YData', y_trace_joint1);
        
        % Display current time
        title(sprintf('2DOF Manipulator Animation - Time: %.2f s', t(i)), 'FontSize', 14);
        
        % Pause for smoother animation
        drawnow;
        pause(0.01);
    end
end