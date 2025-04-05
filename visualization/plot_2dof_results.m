function plot_adaptive_smc_results(t, q, q_d, tau, tau_eq, tau_rob, disturbance, e_all, e_dot_all, s_all, e_norm, s_norm, rho_hat_history, animate, l1, l2, plotOptions)
    % Plot results for 2DOF manipulator with adaptive SMC controller
    % Inputs:
    %   t - Time vector
    %   q - Joint positions
    %   q_d - Desired joint positions
    %   tau - Control inputs
    %   tau_eq - Equivalent control
    %   tau_rob - Robust control
    %   disturbance - External disturbance
    %   e_all - Position errors
    %   e_dot_all - Velocity errors
    %   s_all - Sliding variables
    %   e_norm - Norm of position errors
    %   s_norm - Norm of sliding variables
    %   rho_hat_history - History of adaptive gain
    %   animate - Whether to animate the manipulator
    %   l1, l2 - Link lengths
    %   plotOptions - Structure containing boolean flags for each plot type
    %       .tracking - Tracking performance plots
    %       .errors - Error and sliding variable plots
    %       .control - Control component plots
    %       .disturbance - Disturbance and compensation plots
    %       .adaptiveGain - Adaptive gain evolution plot
    %       .animation - 2DOF manipulator animation
    %       .saveVideo - Whether to save animation as video

    % Set default plot options if not provided
    if nargin < 17
        plotOptions = struct(...
            'tracking', true, ...
            'errors', true, ...
            'control', true, ...
            'disturbance', true, ...
            'adaptiveGain', true, ...
            'animation', animate, ...
            'saveVideo', false);
    end

    %% Plotting Results
    if plotOptions.tracking
        figure('Name', 'Tracking Performance', 'Position', [100, 100, 800, 600]);

        % Joint Position Tracking
        subplot(2,2,1);
        plot(t, q_d(1,:), 'r--', 'LineWidth', 2);
        hold on;
        plot(t, q(1,:), 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Position (rad)', 'FontSize', 12);
        title('Joint 1 Position', 'FontSize', 14);
        legend('Desired', 'Actual', 'FontSize', 10);
        grid on;

        subplot(2,2,2);
        plot(t, q_d(2,:), 'r--', 'LineWidth', 2);
        hold on;
        plot(t, q(2,:), 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Position (rad)', 'FontSize', 12);
        title('Joint 2 Position', 'FontSize', 14);
        legend('Desired', 'Actual', 'FontSize', 10);
        grid on;

        % Control Signals
        subplot(2,2,3);
        plot(t, tau(1,:), 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Torque (N·m)', 'FontSize', 12);
        title('Control Input (Joint 1)', 'FontSize', 14);
        grid on;

        subplot(2,2,4);
        plot(t, tau(2,:), 'b-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Torque (N·m)', 'FontSize', 12);
        title('Control Input (Joint 2)', 'FontSize', 14);
        grid on;
    end

    %% Error and Sliding Variable
    if plotOptions.errors
        figure('Name', 'Tracking Errors', 'Position', [150, 150, 800, 600]);

        % Position errors
        subplot(3,1,1);
        plot(t, e_all(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, e_all(2,:), 'r-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Position Error (rad)', 'FontSize', 12);
        title('Joint Position Errors', 'FontSize', 14);
        legend('e_1', 'e_2', 'FontSize', 10);
        grid on;

        % Velocity errors
        subplot(3,1,2);
        plot(t, e_dot_all(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, e_dot_all(2,:), 'r-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Velocity Error (rad/s)', 'FontSize', 12);
        title('Joint Velocity Errors', 'FontSize', 14);
        legend('ė_1', 'ė_2', 'FontSize', 10);
        grid on;

        % Sliding variables
        subplot(3,1,3);
        plot(t, s_all(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, s_all(2,:), 'r-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Sliding Variable', 'FontSize', 12);
        title('Sliding Variables', 'FontSize', 14);
        legend('s_1', 's_2', 'FontSize', 10);
        grid on;
    end

    %% Control Components
    if plotOptions.control
        figure('Name', 'Control Components', 'Position', [200, 150, 800, 600]);

        % Joint 1 Control Components
        subplot(2,1,1);
        plot(t, tau_eq(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, tau_rob(1,:), 'g-', 'LineWidth', 1.5);
        plot(t, tau(1,:), 'k--', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Torque (N·m)', 'FontSize', 12);
        title('Joint 1 Control Components', 'FontSize', 14);
        legend('τ_{eq} (Equivalent)', 'τ_{rob} (Robust)', 'τ (Total)', 'FontSize', 10);
        grid on;

        % Joint 2 Control Components
        subplot(2,1,2);
        plot(t, tau_eq(2,:), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(t, tau_rob(2,:), 'g-', 'LineWidth', 1.5);
        plot(t, tau(2,:), 'k--', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Torque (N·m)', 'FontSize', 12);
        title('Joint 2 Control Components', 'FontSize', 14);
        legend('τ_{eq} (Equivalent)', 'τ_{rob} (Robust)', 'τ (Total)', 'FontSize', 10);
        grid on;
    end

    %% Disturbance and Compensation
    if plotOptions.disturbance
        figure('Name', 'Disturbance and Compensation', 'Position', [250, 200, 800, 600]);

        % Joint 1 Disturbance
        subplot(2,1,1);
        plot(t, disturbance(1,:), 'r-', 'LineWidth', 1.5);
        hold on;
        plot(t, tau_rob(1,:), 'g-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Torque (N·m)', 'FontSize', 12);
        title('Joint 1: Disturbance vs. Robust Control', 'FontSize', 14);
        legend('External Disturbance', 'Robust Control', 'FontSize', 10);
        grid on;

        % Joint 2 Disturbance
        subplot(2,1,2);
        plot(t, disturbance(2,:), 'r-', 'LineWidth', 1.5);
        hold on;
        plot(t, tau_rob(2,:), 'g-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Torque (N·m)', 'FontSize', 12);
        title('Joint 2: Disturbance vs. Robust Control', 'FontSize', 14);
        legend('External Disturbance', 'Robust Control', 'FontSize', 10);
        grid on;
    end

    %% Adaptive Gain
    if plotOptions.adaptiveGain
        figure('Name', 'Adaptive Gain', 'Position', [300, 250, 800, 500]);
        plot(t, rho_hat_history(1,:), 'b-', 'LineWidth', 2);
        hold on;
        plot(t, rho_hat_history(2,:), 'r-', 'LineWidth', 2);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Adaptive Gain (ρ_{hat})', 'FontSize', 12);
        title('Evolution of Adaptive Gain', 'FontSize', 14);
        legend('Joint 1', 'Joint 2', 'FontSize', 10);
        grid on;
        
        % Add a subplot for sliding variable norm
        axes('Position', [0.65, 0.2, 0.25, 0.2]);
        plot(t, s_norm, 'k-', 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontSize', 10);
        ylabel('||s||', 'FontSize', 10);
        title('Sliding Variable Norm', 'FontSize', 12);
        grid on;
    end

    %% End-Effector Trajectory
    figure('Name', 'End-Effector Trajectory', 'Position', [350, 300, 700, 500]);
    
    % Calculate end-effector positions for actual and desired trajectories
    x_ee = zeros(1, length(t));
    y_ee = zeros(1, length(t));
    x_ee_d = zeros(1, length(t));
    y_ee_d = zeros(1, length(t));
    
    for i = 1:length(t)
        % Actual trajectory
        x_ee(i) = l1*cos(q(1,i)) + l2*cos(q(1,i) + q(2,i));
        y_ee(i) = l1*sin(q(1,i)) + l2*sin(q(1,i) + q(2,i));
        
        % Desired trajectory
        x_ee_d(i) = l1*cos(q_d(1,i)) + l2*cos(q_d(1,i) + q_d(2,i));
        y_ee_d(i) = l1*sin(q_d(1,i)) + l2*sin(q_d(1,i) + q_d(2,i));
    end
    
    % Plot end-effector trajectories
    plot(x_ee_d, y_ee_d, 'r--', 'LineWidth', 2);
    hold on;
    plot(x_ee, y_ee, 'b-', 'LineWidth', 1.5);
    xlabel('X Position (m)', 'FontSize', 12);
    ylabel('Y Position (m)', 'FontSize', 12);
    title('End-Effector Trajectory', 'FontSize', 14);
    legend('Desired', 'Actual', 'FontSize', 10);
    axis equal;
    grid on;

    %% Animate the 2DOF Manipulator
    if plotOptions.animation
        % Check if saveVideo option exists, default to false if not
        if ~isfield(plotOptions, 'saveVideo')
            plotOptions.saveVideo = false;
        end
        animate_2dof_manipulator(t, q, q_d, l1, l2, e_norm, plotOptions.saveVideo);
    end
end

function animate_2dof_manipulator(t, q, q_d, l1, l2, e_norm, saveVideo)
    % Animate the 2DOF manipulator
    % Inputs:
    %   t - Time vector
    %   q - Joint positions
    %   q_d - Desired joint positions
    %   l1, l2 - Link lengths
    %   e_norm - Norm of position errors
    %   saveVideo - Whether to save as video file (optional)
    
    % Default saveVideo to false if not provided
    if nargin < 7
        saveVideo = false;
    end
    
    % Setup video writer if saving video
    if saveVideo
        % Create a timestamp for the filename
        videoFilename = sprintf('video/2dof_manipulator.mp4');
        
        % Create video writer object
        videoFPS = 30;
        videoQuality = 100; % Maximum quality
        videoObj = VideoWriter(videoFilename, 'MPEG-4');
        videoObj.FrameRate = videoFPS;
        videoObj.Quality = videoQuality;
        
        % Open the video file
        open(videoObj);
        fprintf('Recording video to: %s\n', videoFilename);
    end
    
    figure('Name', 'Manipulator Animation', 'Position', [100, 100, 800, 600]);
    
    % Set axis limits with some margin
    axis_limit = (l1 + l2) * 1.1;
    ax = axes;
    hold(ax, 'on');
    xlim([-axis_limit, axis_limit]);
    ylim([-axis_limit, axis_limit]);
    axis equal;
    grid on;
    title('2DOF Manipulator Animation');
    xlabel('X (m)');
    ylabel('Y (m)');
    
    % Create graphics objects for the links and joints
    link1_line = line('XData', [0, 0], 'YData', [0, 0], 'Color', 'blue', 'LineWidth', 3);
    link2_line = line('XData', [0, 0], 'YData', [0, 0], 'Color', 'red', 'LineWidth', 3);
    joint0_marker = plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    joint1_marker = plot(0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    ee_marker = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Calculate and plot reference trajectory
    x_ref = zeros(1, length(t));
    y_ref = zeros(1, length(t));
    for i = 1:length(t)
        % Forward kinematics for desired trajectory
        x_ref(i) = l1*cos(q_d(1,i)) + l2*cos(q_d(1,i) + q_d(2,i));
        y_ref(i) = l1*sin(q_d(1,i)) + l2*sin(q_d(1,i) + q_d(2,i));
    end
    reference_path = plot(x_ref, y_ref, 'g--', 'LineWidth', 1.5);
    
    % Create trace arrays to store the actual path
    x_trace = [];
    y_trace = [];
    trace_line = plot(0, 0, 'b-', 'LineWidth', 1.5);
    
    % Create trace arrays for first joint
    x_trace_joint1 = [];
    y_trace_joint1 = [];
    trace_line_joint1 = plot(0, 0, 'c-', 'LineWidth', 1.5);
    
    % Animation speed control
    skip_frames = 10;
    
    legend('Link 1', 'Link 2', 'Base', 'Joint 1', 'End Effector', 'Desired Path', 'End-Effector Path', 'Joint 1 Path');
    
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
        
        % Display current time and error
        title(sprintf('2DOF Manipulator Animation - Time: %.2f s - Error: %.4f', t(i), e_norm(i)));
        
        % Save the current frame to video if requested
        if saveVideo
            frame = getframe(gcf);
            writeVideo(videoObj, frame);
        end
        
        % Pause for smoother animation
        drawnow;
        pause(0.01);
    end
    
    % Close the video file if it was opened
    if saveVideo
        close(videoObj);
        fprintf('Video saved successfully to: %s\n', videoFilename);
    end
end 