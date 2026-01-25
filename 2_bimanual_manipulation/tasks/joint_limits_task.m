classdef joint_limits_task < Task
    properties
        %Security buffer around joint limits
        buffer = 0.17; 
    end

    methods
        function obj = joint_limits_task(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end

        function updateReference(obj, robot_system)
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;    
            end

            %  initialize reference velocity vector
            obj.xdotbar = zeros(7,1);
            
            % Gain for repulsion effect
            lambda = 1.0; 

            for i = 1:7
                q = robot.q(i);
                q_min = robot.jlmin(i);
                q_max = robot.jlmax(i);
                
                % Activation thresholds
                % danger zone MIN: da q_min a (q_min + buffer)
                safe_min = q_min + obj.buffer;
                
                % danger zone MAX: da (q_max - buffer) a q_max
                safe_max = q_max - obj.buffer;

                % Repulsion logic
                % if we are too close to the minimum, push towards safe_min
                if q < safe_min
                    
                    obj.xdotbar(i) = lambda * (safe_min - q);
                
                % if we are too close to the maximum, push towards safe_max
                elseif q > safe_max
                    
                    obj.xdotbar(i) = lambda * (safe_max - q);
                else
                    obj.xdotbar(i) = 0;
                end
            end

            % Safety saturation to avoid jumps
            obj.xdotbar = Saturate(obj.xdotbar, 1.0);
        end

        function updateActivation(obj, robot_system)
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;    
            end

            
            % A(i,i) = 1 if close to joint limit, 0 otherwise
            activations = zeros(7,1);

            for i = 1:7
                q = robot.q(i);
                q_min = robot.jlmin(i);
                q_max = robot.jlmax(i);
                
                
                % DecreasingBell for inferior limit
                a_min = DecreasingBellShapedFunction(q_min, q_min + obj.buffer, 0, 1, q);
                
                % IncreasingBell for superior limit
                a_max = IncreasingBellShapedFunction(q_max - obj.buffer, q_max, 0, 1, q);
                
                % The total activation is the sum (they don't overlap)
                activations(i) = a_min + a_max;
            end
            
            % Diagonal activation matrix
            obj.A = diag(activations);
        end

        function updateJacobian(obj, robot_system)
            
            if obj.ID == 'L'
                % If we are the left arm, we act on the first 7 dof
                obj.J = [eye(7), zeros(7,7)];
            elseif obj.ID == 'R'
                % If we are the right arm, we act on the last 7 dof
                obj.J = [zeros(7,7), eye(7)];
            end
        end
    end
end