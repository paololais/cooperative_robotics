classdef TaskVehicleHeading < Task   
    methods
        function updateReference(obj, robot)

            % w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
            % w_vehicle_goal_position = [10.5 37.5 -38]';
            % Vehicle x-axis
            w_xv = robot.wTv(1:3,1);

            % Nodule position
            nodule_pos = robot.wTg(1:3,4);

            % Vector from vehicle to nodule (horizontal plane)
            diff = nodule_pos - robot.wTv(1:3,4);
            diff(3) = 0;
            if norm(diff) > 1e-3
                w_xd = diff / norm(diff);
            else
                w_xd = w_xv;  % avoid singularity
            end

            % Rotation axis
            n = cross(w_xv, w_xd);
            sin_theta = norm(n);
            if sin_theta > 1e-6
                n = n / sin_theta;
            else
                n = zeros(3,1);
            end

            % Signed angle
            cos_theta = dot(w_xv, w_xd);
            theta = atan2(sin_theta, cos_theta);

            % Threshold
            theta_min = 0.05;
            if abs(theta) < theta_min
                obj.xdotbar = zeros(3,1);
            else
                kp = 0.6;
                obj.xdotbar = kp * theta * n;  % 3x1 vector
                obj.xdotbar = Saturate(obj.xdotbar, 0.5);
            end

            robot.theta_error = theta;  % for logging
        end

        function updateJacobian(obj, robot)
            % Map vehicle angular velocities to heading task
            J = zeros(3, 13);
            J(:,11:13) = eye(3);  % last 3 vehicle DOFs = angular velocities
            obj.J = J;
        end

        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end
