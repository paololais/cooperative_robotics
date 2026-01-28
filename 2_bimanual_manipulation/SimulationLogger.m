classdef SimulationLogger < handle
    properties
        t            % time vector
        ql            % joint positions
        qdotl        % joint velocities
        qr            % joint positions
        qdotr        % joint velocities
        a            % task activations (diagonal only)
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        action_set     % set of actions
        n
        dist_tools

        xdot_desiredL
        xdot_desiredR
        xdot_actualL
        xdot_actualR
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, action_set)
            obj.robot = robotModel;
            obj.action_set = action_set;

            obj.t = zeros(1, maxLoops);
            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);
            obj.dist_tools = zeros(1, maxLoops);

            obj.xdot_actualL = zeros(6, maxLoops);
            obj.xdot_actualR = zeros(6, maxLoops);
            obj.xdot_desiredL = zeros(6, maxLoops);
            obj.xdot_desiredR = zeros(6, maxLoops);
            
            obj.n=length(action_set.actions);
            l=zeros(1,obj.n);
            for i=1:obj.n
                l(i)=length(action_set.actions{i});
            end

            obj.xdotbar_task=cell(length(action_set.actions), max(l), maxLoops);
            
        end

        function update(obj, t, loop, missionPhase)
            % Store robot state
            obj.t(loop) = t;
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;

            % 2. Metrics
            obj.dist_tools(loop) = norm(obj.robot.left_arm.wTt(1:3, 4) - obj.robot.right_arm.wTt(1:3, 4));
            if missionPhase == 1
                Jl = obj.robot.left_arm.wJt;
                Jr = obj.robot.right_arm.wJt;
            elseif missionPhase == 2 || missionPhase == 3
                Jl = obj.robot.left_arm.wJo;
                Jr = obj.robot.right_arm.wJo;
            end
            obj.xdot_actualL(:,loop) = Jl * obj.robot.left_arm.qdot;
            obj.xdot_actualR(:,loop) = Jr * obj.robot.right_arm.qdot;
            obj.xdot_desiredL(:,loop) = obj.robot.left_arm.xdot_des;
            obj.xdot_desiredR(:,loop) = obj.robot.right_arm.xdot_des;

            %Store task reference velocities
            for i=1:obj.n
                for j=1:length(obj.action_set.actions{i})
                    obj.xdotbar_task{i,j,loop}=obj.action_set.actions{i}{j}.xdotbar;
                end
            end


        end
        function plotAll(obj,action,task)
            % Example plotting for robot state
            figure(1);
            subplot(2,1,1);
            title('Left arm motion');
            plot(obj.t, obj.ql, 'LineWidth', 2);
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            subplot(2,1,2);
            plot(obj.t, obj.qdotl, 'LineWidth', 2);
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');
            figure(2);
            title('Right arm motion');    
            subplot(2,1,1);
            plot(obj.t, obj.qr, 'LineWidth', 2);
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            subplot(2,1,2);
            plot(obj.t, obj.qdotr, 'LineWidth', 2);
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');

            % Optional: plot a number of tasks from an specific action set
            figure(3);
            s=squeeze(obj.xdotbar_task(action,:,:));
            nt=length(task);
            title(strcat('Action Set'," ",num2str(action)));  
            for i=1:nt
                subplot(1,nt,i)
                data=cell2mat(s(task(i),:));
                plot(obj.t(1,1:end-1),data','.-')
                grid on
                if(obj.action_set.actions{action}{task(i)}.task_name=="T")
                    legend('wx','wy','wz','vx','vy','vz');
                end
                title([strcat('Robot'," ",obj.action_set.actions{action}{task(i)}.ID," ",'Task'," ",num2str(task(i))," ",obj.action_set.actions{action}{task(i)}.task_name)]);
            end
        end
            
        function plotQ4(obj)
            % PLOT 4: Constraint (Standard)
            figure('Name', 'Report: Constraint');
            plot(obj.t, obj.dist_tools, 'LineWidth', 2);
            grid on; xlabel('Time [s]'); ylabel('Distance [m]');
            title('Distance between Tool Frames');
            yline(mean(obj.dist_tools(obj.t>2)), '--r');

            figure('Name', 'Velocity Comparison');            
            titles = {'v_x [m/s]', 'v_y [m/s]', 'v_z [m/s]', ...
                      '\omega_x [rad/s]', '\omega_y [rad/s]', '\omega_z [rad/s]'};
            
            for i = 1:6
                subplot(3, 2, i);
                hold on;
                % 1. Desired Object Velocity (Reference)
                h1 = plot(obj.t, obj.xdot_desiredL(i, :), 'g', 'LineWidth', 1.5);
                
                % 2. left arm
                h2 = plot(obj.t, obj.xdot_actualL(i, :), 'r', 'LineWidth', 1.0);
                
                % 3. right arm
                h3 = plot(obj.t, obj.xdot_actualR(i, :), 'b', 'LineWidth', 1.0);
                
                ylabel(titles{i});
                if i > 4, xlabel('Time [s]'); end
                grid on;
                
                % Add vertical bars/background to indicate Phase 2 (Cooperation)
                % Assuming Phase 2 is roughly between t=2 and t=8 (adjust based on your sim)
                % xline(2.0, '--k', 'Start Coop'); 
                
                if i == 2
                    legend([h1, h2, h3], 'Desired (Ref)', 'left-arm', 'right-arm', ...
                           'Location', 'bestoutside');
                end
            end
            sgtitle('Velocity Comparison');
        end
    end
end