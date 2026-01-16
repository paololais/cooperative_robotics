classdef object_motion_task < Task
    % Object Cartesian motion task (after grasp)

    methods
        function obj = object_motion_task(taskID)
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            % Use LEFT arm as object pose reference
            left_arm = robot_system.left_arm;

            % Current and desired object pose
            wTo  = left_arm.wTo;
            wTog = left_arm.wTog;

            % Cartesian error
            [v_ang, v_lin] = CartError(wTog, wTo);

            % Desired object velocity
            obj.xdotbar = [v_ang; v_lin];

            % Saturation
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end
        
        function updateJacobian(obj, robot_system)
            % Object Jacobian: both arms contribute
            JL = robot_system.left_arm.wJo;
            JR = robot_system.right_arm.wJo;

            obj.J = [JL, JR];   % 6x14
        end
        
        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end
