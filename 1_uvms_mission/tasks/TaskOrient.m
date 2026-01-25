classdef TaskOrient < Task
    % Vehicle attitude control task
    properties
        error
    end

    methods
        function updateReference(obj, robot)
            [w_ang, ~] = CartError(robot.wTgv, robot.wTv);
            
            obj.xdotbar = 0.5 * w_ang;
            obj.xdotbar = Saturate(obj.xdotbar, 0.4);
            obj.error = norm(w_ang);
        end

        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3);
            obj.J = [zeros(3,7), zeros(3,3), wRv];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end