classdef TaskVehicleHeading < Task  
% Vehicle longitudinal alignment to nodule task
    properties
        n = zeros(3,1);
        d = zeros(3,1);
    end

    methods
        function updateReference(obj, robot)
            % Vehicle x-axis
            w_xv = robot.wTv(1:3,1);

            % Vector from vehicle to nodule (horizontal plane)
            obj.d = robot.wTg(1:3,4) - robot.wTv(1:3,4);
            obj.d(3) = 0; % neglect vertical component
            if norm(obj.d) > 1e-3
                w_xd = obj.d / norm(obj.d);
            else
                w_xd = w_xv;  % avoid singularity
            end

            % Rotation axis
            obj.n = cross(w_xv, w_xd);
            sin_theta = norm(obj.n);
            if sin_theta > 1e-6
                obj.n = obj.n / sin_theta;
            else
                obj.n = zeros(3,1);
            end

            % Signed angle
            cos_theta = dot(w_xv, w_xd);
            theta = atan2(sin_theta, cos_theta);

            obj.xdotbar = - 0.4 * theta;
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);

            robot.theta_error = theta;  % store heading error in robot model
        end

        function updateJacobian(obj, robot)
            % Jacobian maps vehicle angular velocities to heading change
            obj.J = obj.n' * [zeros(3,7) -skew(obj.d)/(norm(obj.d))^2 -eye(3)];
        end

        function updateActivation(obj, robot)
            % Inequality task
            if isempty(robot.theta_error)
                theta = 0;
            else
                theta = robot.theta_error;
            end            
            obj.A = IncreasingBellShapedFunction(0.05, 0.15, 0, 1, abs(theta));
        end
    end
end
