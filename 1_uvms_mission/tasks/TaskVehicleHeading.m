classdef TaskVehicleHeading < Task   
    properties
        % Non serve più nessuna proprietà extra!
    end

    methods
        function updateReference(obj, robot)
            % 1. Vettore di interesse del veicolo: Asse X
            w_xv = robot.wTv(1:3,1);
            
            % 2. Recupera la posizione del nodulo direttamente dal modello
            % wTg è 4x4, prendiamo la traslazione (prime 3 righe, colonna 4)
            nodule_pos = robot.wTg(1:3, 4); 
            
            % Vettore dal veicolo al nodulo
            diff = nodule_pos - robot.wTv(1:3,4);
            diff(3) = 0; % Proiezione sul piano orizzontale
            
            % Calcolo versore desiderato
            if norm(diff) > 0.01
                w_xd = diff / norm(diff);
            else
                w_xd = w_xv; % Evita singolarità
            end
        
            % 3. Calcolo errore di disallineamento
            n = cross(w_xd, w_xv);           
            sin_theta = norm(n);             
            cos_theta = dot(w_xd, w_xv);     
            theta = atan2(sin_theta, cos_theta);

            robot.theta_error = theta; 
            
            % 4. Controllo
            obj.xdotbar = 1.0 * (0 - theta);
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);
        end

        function updateJacobian(obj, robot)
            w_xv = robot.wTv(1:3,1);
            
            % Recupera nodulo anche qui
            nodule_pos = robot.wTg(1:3, 4);
            
            diff = nodule_pos - robot.wTv(1:3,4);
            diff(3) = 0;
            if norm(diff) > 0.01
                w_xd = diff / norm(diff);
            else
                w_xd = w_xv;
            end
            
            n = cross(w_xv, w_xd);
            if norm(n) > 0
                n = n/norm(n);
            end
            
            obj.J = n' * [zeros(3,7) zeros(3) eye(3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = 1;
        end
    end
end