classdef joint_limits_task < Task
    properties
        % Buffer di sicurezza (es. 10 gradi in radianti)
        % Il task inizia ad attivarsi quando mancano 0.17 rad al limite
        buffer = 0.17; 
    end

    methods
        function obj = joint_limits_task(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end

        function updateReference(obj, robot_system)
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;    
            end

            % Inizializza il vettore delle velocità richieste (7x1)
            obj.xdotbar = zeros(7,1);
            
            % Guadagno del controllo (quanto forte respingiamo)
            lambda = 1.0; 

            for i = 1:7
                q = robot.q(i);
                q_min = robot.jlmin(i);
                q_max = robot.jlmax(i);
                
                % Soglie di attivazione
                % Zona pericolo MIN: da q_min a (q_min + buffer)
                safe_min = q_min + obj.buffer;
                
                % Zona pericolo MAX: da (q_max - buffer) a q_max
                safe_max = q_max - obj.buffer;

                % --- LOGICA DI REPULSIONE ---
                % Se siamo troppo vicini al minimo, spingiamo verso safe_min
                if q < safe_min
                    % Esempio: q = -2.8, safe = -2.7. 
                    % Error = -2.7 - (-2.8) = +0.1 (Spinge POSITIVO -> OK)
                    obj.xdotbar(i) = lambda * (safe_min - q);
                
                % Se siamo troppo vicini al massimo, spingiamo verso safe_max
                elseif q > safe_max
                    % Esempio: q = 2.8, safe = 2.7.
                    % Error = 2.7 - 2.8 = -0.1 (Spinge NEGATIVO -> OK)
                    obj.xdotbar(i) = lambda * (safe_max - q);
                else
                    obj.xdotbar(i) = 0;
                end
            end
            
            % Saturazione di sicurezza per evitare scatti
            obj.xdotbar = Saturate(obj.xdotbar, 1.0);
        end

        function updateActivation(obj, robot_system)
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;    
            end

            % L'attivazione A deve essere una matrice diagonale 7x7
            % A(i,i) = 1 se il giunto i è in pericolo, 0 se è sicuro.
            activations = zeros(7,1);

            for i = 1:7
                q = robot.q(i);
                q_min = robot.jlmin(i);
                q_max = robot.jlmax(i);
                
                % Definiamo le zone per le Bell Functions
                % Min: Attivo pienamente a q_min, inizia a 0 a (q_min + buffer)
                % Max: Attivo pienamente a q_max, inizia a 0 a (q_max - buffer)
                
                % DecreasingBell per il limite inferiore
                a_min = DecreasingBellShapedFunction(q_min, q_min + obj.buffer, 0, 1, q);
                
                % IncreasingBell per il limite superiore
                a_max = IncreasingBellShapedFunction(q_max - obj.buffer, q_max, 0, 1, q);
                
                % L'attivazione totale è la somma (non si sovrappongono mai)
                activations(i) = a_min + a_max;
            end
            
            % Costruisci la matrice diagonale
            obj.A = diag(activations);
        end

        function updateJacobian(obj, robot_system)
            % Il compito agisce direttamente nello spazio dei giunti (Joint Space).
            % Quindi lo Jacobiano "locale" è una matrice identità 7x7.
            % Dobbiamo però mapparla sul sistema completo (14 dof).
            
            if obj.ID == 'L'
                % Se siamo il braccio sinistro, agiamo sui primi 7 dof
                obj.J = [eye(7), zeros(7,7)];
            elseif obj.ID == 'R'
                % Se siamo il braccio destro, agiamo sugli ultimi 7 dof
                obj.J = [zeros(7,7), eye(7)];
            end
        end
    end
end