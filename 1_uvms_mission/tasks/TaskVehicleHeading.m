classdef TaskVehicleHeading < Task   
    properties
        target_point = [0; 0; 0]; % Proprietà da settare nel main
    end

    methods
        % Metodo per impostare il target dall'esterno
        function setTarget(obj, point)
            obj.target_point = point;
        end

        function updateReference(obj, robot)
            % 1. Vettore di interesse del veicolo: Asse X
            w_xv = robot.wTv(1:3,1);
            
            % 2. Vettore target (World): Direzione verso il target_point
            diff = obj.target_point - robot.wTv(1:3,4);
            diff(3) = 0; % Proiezione sul piano orizzontale
            
            % Calcolo versore desiderato
            if norm(diff) > 0.01
                w_xd = diff / norm(diff);
            else
                w_xd = w_xv; % Evita singolarità
            end
        
            % 3. Calcolo errore di disallineamento (tra X attuale e X desiderata)
            n = cross(w_xd, w_xv);           
            sin_theta = norm(n);             
            cos_theta = dot(w_xd, w_xv);     
            theta = atan2(sin_theta, cos_theta);
            
            % 4. Velocità di riferimento
            % Controllo proporzionale per annullare theta
            obj.xdotbar = 1.5 * (0 - theta);
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);
        end

        function updateJacobian(obj, robot)
            w_xv = robot.wTv(1:3,1);
            
            diff = obj.target_point - robot.wTv(1:3,4);
            diff(3) = 0;
            if norm(diff) > 0.01
                w_xd = diff / norm(diff);
            else
                w_xd = w_xv;
            end
            
            % Asse di rotazione
            n = cross(w_xv, w_xd);
            if norm(n) > 0
                n = n/norm(n);
            end
            
            % Jacobiano angolare
            obj.J = n' * [zeros(3,7) zeros(3) eye(3)];
        end
        
        function updateActivation(obj, robot)
            % Attivazione costante per mantenere il puntamento
            obj.A = 1;
        end
    end
end