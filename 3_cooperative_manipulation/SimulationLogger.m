classdef SimulationLogger < handle
    properties
        t            % time vector
        q            % joint positions
        qdot         % joint velocities
        
        % NEW: Velocity Analysis properties
        xdot_desired % Desired Object Reference Velocity (6xN)
        xdot_nc      % Non-Cooperative Cartesian Velocity (6xN)
        xdot_actual  % Actual/Cooperative Cartesian Velocity (6xN)
        
        % Tool distance tracking
        tool_pos_left   % Left tool position (3xN)
        tool_pos_right  % Right tool position (3xN)
        tool_distance   % Distance between tools (1xN)
        
        robot        % robot model
    end

    methods
        function obj = SimulationLogger(maxLoops, robot)
            obj.robot = robot;

            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.qdot = zeros(7, maxLoops);
            
            % Initialize new storage fields
            obj.xdot_desired = zeros(6, maxLoops);
            obj.xdot_nc = zeros(6, maxLoops);
            obj.xdot_actual = zeros(6, maxLoops);
            
            % Initialize tool distance tracking
            obj.tool_pos_left = zeros(3, maxLoops);
            obj.tool_pos_right = zeros(3, maxLoops);
            obj.tool_distance = zeros(1, maxLoops);
        end

        % Update function now requires extra arguments for the velocities
        function update(obj, t, loop, xdot_ref, xdot_nc_val)
            % Store basic robot state
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.qdot(:, loop) = obj.robot.qdot;
            
            % Store velocity analysis data
            obj.xdot_desired(:, loop) = xdot_ref;
            obj.xdot_nc(:, loop) = xdot_nc_val;
            
            % Compute actual Cartesian velocity (J * qdot)
            % This represents the final Cooperative velocity executed by the robot
            obj.xdot_actual(:, loop) = obj.robot.wJt * obj.robot.qdot;
        end
        
        % Update function for dual-arm scenarios to track tool distance
        function updateDualArm(obj, t, loop, left_arm, right_arm, xdot_ref, xdot_nc_val)
            % Store basic robot state
            obj.t(loop) = t;
            
            % Store velocity analysis data
            obj.xdot_desired(:, loop) = xdot_ref;
            obj.xdot_nc(:, loop) = xdot_nc_val;
            
            % Extract tool positions
            left_pos = left_arm.wTt(1:3, 4);   % Left tool position
            right_pos = right_arm.wTt(1:3, 4);  % Right tool position

            obj.tool_pos_left(:, loop) = left_pos;
            obj.tool_pos_right(:, loop) = right_pos;
            
            % Compute distance between tools
            obj.tool_distance(loop) = norm(left_pos - right_pos);
        end

        function plotAll(obj)
            % Standard plot for Joint positions and velocities
            figure('Name', 'Joint States');
            subplot(2,1,1);
            plot(obj.t, obj.q,'.-','LineWidth', 1);
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            title('Joint positions');
            grid on;
            
            subplot(2,1,2);
            plot(obj.t, obj.qdot,'.-', 'LineWidth', 1);
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');
            title('Joint velocities');
            grid on;
        end

        function plotVelocityComparison(obj, armName)
            figure('Name', [armName ' - Velocity Comparison']);
            
            titles = {'v_x [m/s]', 'v_y [m/s]', 'v_z [m/s]', ...
                      '\omega_x [rad/s]', '\omega_y [rad/s]', '\omega_z [rad/s]'};
            
            for i = 1:6
                subplot(3, 2, i);
                hold on;
                % 1. Desired Object Velocity (Reference) - Black Dashed
                h1 = plot(obj.t, obj.xdot_desired(i, :), 'g', 'LineWidth', 1.5);
                
                % 2. Non-Cooperative Velocity - Red
                h2 = plot(obj.t, obj.xdot_nc(i, :), 'r', 'LineWidth', 1.0);
                
                % 3. Actual Cooperative Velocity - Blue
                h3 = plot(obj.t, obj.xdot_actual(i, :), 'b', 'LineWidth', 1.2);
                
                ylabel(titles{i});
                if i > 4, xlabel('Time [s]'); end
                grid on;
                
                % Add vertical bars/background to indicate Phase 2 (Cooperation)
                % Assuming Phase 2 is roughly between t=2 and t=8 (adjust based on your sim)
                % xline(2.0, '--k', 'Start Coop'); 
                
                if i == 2
                    legend([h1, h2, h3], 'Desired (Ref)', 'Non-Cooperative', 'Cooperative (Actual)', ...
                           'Location', 'bestoutside');
                end
            end
            sgtitle([armName ' Velocity Comparison']);
        end
        
        function plotToolDistance(obj)
            figure('Name', 'Tool-to-Tool Distance');
            
            plot(obj.t, obj.tool_distance, 'b', 'LineWidth', 2);
            xlabel('Time [s]');
            ylabel('Distance [m]');
            title('Distance Between Left and Right Tool End-Effectors');
            grid on;
            
            % Add some statistics to the plot
            hold on;
            min_dist = min(obj.tool_distance);
            max_dist = max(obj.tool_distance);
            mean_dist = mean(obj.tool_distance);
            
            yline(mean_dist, '--r', sprintf('Mean: %.3f m', mean_dist), 'LineWidth', 1.5);
            yline(min_dist, '--g', sprintf('Min: %.3f m', min_dist), 'LineWidth', 1.5);
            yline(max_dist, '--m', sprintf('Max: %.3f m', max_dist), 'LineWidth', 1.5);
            
            legend('Distance', 'Mean', 'Min', 'Max', 'Location', 'best');
        end
    end
end