classdef ee_min_altitude_task < Task   
    %Tool position control for a single arm
    properties

    end

    methods
        function obj=ee_min_altitude_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end

        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            
            error = 0.15 - robot.alt; % minimum altitude = 0.15m
            obj.xdotbar = 0.6 * max(error, 0); % only push upward
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
        end

        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            tool_jacobian_lin=robot.wJt(1:3,:);
            
            if obj.ID=='L'
                obj.J=[0 0 1] * [tool_jacobian_lin, zeros(3, 7)];
            elseif obj.ID=='R'
                obj.J=[0 0 1] * [zeros(3, 7), tool_jacobian_lin];
            end
        end

        function updateActivation(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end

            alt = robot.alt;
            % start activating when alt < 0.18m, 
            % fully active when alt <= 0.15m
            obj.A = DecreasingBellShapedFunction(0.15, 0.18, 0, 1, alt);
        end
    end
end