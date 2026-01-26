classdef coop_tool_velocity_task < Task
    % Non-reactive cooperative tool velocity tracking

    methods
        function obj = coop_tool_velocity_task(robotID, taskID)
            obj.ID = robotID;
            obj.task_name = taskID;
        end

        function updateReference(obj, robot)
            % reference comes from cooperation layer
            obj.xdotbar = robot.xdot_coop;   % 6x1
        end

        function updateJacobian(obj, robot)
            w_d_to = robot.wTt(1:3,1:3) * robot.tTo(1:3,4);
            tS_objc = [eye(3), zeros(3,3);
                      (skew(w_d_to)'), eye(3)];

            tool_jacobian = robot.wJt;

            obj.J = tS_objc * tool_jacobian;

        end

        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end
