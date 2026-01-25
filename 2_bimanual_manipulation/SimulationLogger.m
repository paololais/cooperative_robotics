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
            obj.n=length(action_set.actions);
            l=zeros(1,obj.n);
            for i=1:obj.n
                l(i)=length(action_set.actions{i});
            end

            obj.xdotbar_task=cell(length(action_set.actions), max(l), maxLoops);
            
        end

        function update(obj, t, loop)
            % Store robot state
            obj.t(loop) = t;
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
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


            % ---------------------------------------------------------
        % UPDATED PLOTTING FUNCTION (Includes Manipulability & Joint Vels)
        % ---------------------------------------------------------
        function plotQ4(obj)
            fprintf('Replaying simulation data for Report plots...\n');
            
            % Setup
            valid_steps = find(obj.t > 0, 1, 'last');
            if isempty(valid_steps), valid_steps = length(obj.t); end
            t_plot = obj.t(1:valid_steps);
            
            % Data Arrays
            dist_tools = zeros(1, valid_steps);
            vel_L_norm = zeros(1, valid_steps);
            vel_R_norm = zeros(1, valid_steps);
            vel_obj_ref_norm = zeros(1, valid_steps);
            manip_L = zeros(1, valid_steps); % Manipulability Left
            manip_R = zeros(1, valid_steps); % Manipulability Right
            
            idx_action = 2;       
            idx_task_obj = 6;     
    
            for k = 1:valid_steps
                % 1. Update Kinematics
                obj.robot.left_arm.q = obj.ql(:, k);
                obj.robot.right_arm.q = obj.qr(:, k);
                obj.robot.update_full_kinematics();
    
                % 2. Metrics
                dist_tools(k) = norm(obj.robot.left_arm.wTt(1:3, 4) - obj.robot.right_arm.wTt(1:3, 4));
                
                v_L = obj.robot.left_arm.wJt * obj.qdotl(:, k);
                v_R = obj.robot.right_arm.wJt * obj.qdotr(:, k);
                vel_L_norm(k) = norm(v_L(1:3));
                vel_R_norm(k) = norm(v_R(1:3));
                
                % 3. Compute Manipulability (Yoshikawa measure)
                % w = sqrt(det(J * J'))
                J_L = obj.robot.left_arm.wJt;
                J_R = obj.robot.right_arm.wJt;
                manip_L(k) = sqrt(det(J_L * J_L'));
                manip_R(k) = sqrt(det(J_R * J_R'));

                % 4. Reference
                try
                    ref_vel = obj.xdotbar_task{idx_action, idx_task_obj, k};
                    if ~isempty(ref_vel), vel_obj_ref_norm(k) = norm(ref_vel(4:6)); end
                catch 
                end
            end
    
            % --- PLOT 4: Manipulability (NEW) ---
            figure('Name', 'Report: Manipulability');
            plot(t_plot, manip_L, 'b', 'LineWidth', 1.5, 'DisplayName', 'Left Arm');
            hold on;
            plot(t_plot, manip_R, 'r', 'LineWidth', 1.5, 'DisplayName', 'Right Arm');
            yline(0, 'k-', 'Singularity');
            grid on; legend;
            xlabel('Time [s]'); ylabel('Manipulability Index');
            title('System Manipulability (Singularity Check)');
            

            % --- PLOT 5: Joint Velocities (NEW) ---
            figure('Name', 'Report: Joint Velocities (Left)');
            plot(t_plot, obj.qdotl(:, 1:valid_steps)', 'LineWidth', 1);
            grid on;
            xlabel('Time [s]'); ylabel('Rad/s');
            title('Left Arm Joint Velocities');
            

            % --- PLOT 6: Constraint (Standard) ---
            figure('Name', 'Report: Constraint');
            plot(t_plot, dist_tools, 'LineWidth', 2);
            grid on; xlabel('Time [s]'); ylabel('Distance [m]');
            title('Distance between Tool Frames');
            yline(mean(dist_tools(t_plot>2)), '--r');
    
            % --- PLOT 7: Velocity Comparison (Standard) ---
            figure('Name', 'Report: Velocity Comparison');
            hold on;
            plot(t_plot, vel_obj_ref_norm, 'r', 'LineWidth', 2, 'DisplayName', 'Desired Obj Vel');
            plot(t_plot, vel_L_norm, 'b', 'LineWidth', 1.5, 'DisplayName', 'Left Tool Vel');
            plot(t_plot, vel_R_norm, 'g', 'LineWidth', 1.5, 'DisplayName', 'Right Tool Vel');
            grid on; legend('Location','best');
            xlabel('Time [s]'); ylabel('Velocity [m/s]');
            title('Velocity Tracking Analysis');
            
            fprintf('All Report plots generated.\n');
        end
    end
end