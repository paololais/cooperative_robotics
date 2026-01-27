classdef ee_min_altitude_task < Task
    properties

    end

    methods
        function obj=ee_min_altitude_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end

        function updateReference(obj, robot)                
            error = 0.2 - robot.alt;
            obj.xdotbar = 1.5 * error;         
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 1.5);
        end

        function updateJacobian(obj,robot)
            tool_jacobian_lin=robot.wJt(1:3,:);            
            obj.J=[0 0 1] * tool_jacobian_lin;
        end

        function updateActivation(obj, robot)
            alt = robot.alt;
            obj.A = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, alt);
        end
    end
end