classdef rigid_constraint_task < Task
    % Bimanual rigid grasp kinematic constraint
    % Enforces: J_oL*qdot_L - J_oR*qdot_R = 0

    methods
        function obj = rigid_constraint_task(taskID)
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            % Rigid constraint → zero reference (6D)
            obj.xdotbar = zeros(6,1);
        end
        
        function updateJacobian(obj, robot_system)
            left_arm  = robot_system.left_arm;
            right_arm = robot_system.right_arm;
            
            % Object-frame Jacobians
            J_oL = left_arm.wJo;   % 6x7
            J_oR = right_arm.wJo;  % 6x7
            
            % Rigid grasp constraint Jacobian (6x14)
            obj.J = [J_oL, -J_oR];
        end
        
        function updateActivation(obj, robot_system)
            % Physical constraint → always active
            obj.A = eye(6);
        end
    end
end
