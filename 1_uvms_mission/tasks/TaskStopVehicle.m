classdef TaskStopVehicle < Task
% Vehicle stop motion task
    methods
        function updateReference(obj, robot)
            obj.xdotbar = zeros(6,1); % 0 final velocity
        end

        function updateJacobian(obj, robot)
            obj.J = [zeros(6,7), eye(6)]; 
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6); 
        end
    end
end