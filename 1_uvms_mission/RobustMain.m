% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('./tasks/')
clc; clear; close all;

% Simulation parameters
dt       = 0.005;
endTime  = 40;
% Initialize robot model and simulator
robotModel = UvmsModel();          
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define tasks    
task_tool = TaskTool();
task_vehicle_pos = TaskVehiclePosition();
task_hor_attitude = TaskHorAttitude();
task_vehicle_alt = TaskVehicleAltitude();
task_vehicle_land = TaskVehicleLand();
task_vehicle_heading = TaskVehicleHeading();
task_stop_vehicle = TaskStopVehicle();

task_set1 = { task_vehicle_alt, task_hor_attitude, task_vehicle_pos };   % Safe Navigation
task_set2 = { task_hor_attitude, task_vehicle_heading, task_vehicle_land, task_vehicle_pos };  % Landing
task_set3 = { task_stop_vehicle, task_tool };  % Manipulation

% Unifying task list
unified_task_list = {task_vehicle_alt, task_stop_vehicle, ... % safety
                    task_hor_attitude, task_vehicle_heading, ... % prerequisites
                    task_vehicle_land, task_vehicle_pos, task_tool }; % action defining

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.addAction(task_set1, "Safe Navigation");  % Action 1: Safe Navigation
actionManager.addAction(task_set2, "Landing");          % Action 2: Landing
actionManager.addAction(task_set3, "Manipulation");     % Action 3: Manipulation
actionManager.addUnifyingTaskList(unified_task_list);

disp(actionManager.actionsName)

% Set current action
actionManager.setCurrentAction("Safe Navigation");

% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]'; % nodule position
w_arm_goal_orientation = [0, pi, pi/2];

% Vehicle goal position and orientation
w_vehicle_goal_position = [10.5 37.5 -38]';
w_vehicle_goal_orientation = [0, -0.06, 0.5];

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set1);
idx=1;

% Initialize mission phase: 1=Safe Navigation, 2=Landing, 3=Manipulation
missionPhase = 1;

% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % Mission controller
    if missionPhase == 1
    %if strcmp(actionManager.actionsName{actionManager.currentAction}, "Safe Navigation")
        % Check if vehicle reached goal position
        xy_error = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));
        if xy_error < 0.2            
            disp("Safe Navigation complete - switch to Landing")
            actionManager.setCurrentAction("Landing");
            missionPhase = 2;
        end
    elseif missionPhase == 2
        alt_error = abs(robotModel.altitude - 0.5);
        if alt_error < 0.1 && abs(robotModel.theta_error) < 0.1
            disp("Landing complete - switch to Manipulation")
            actionManager.setCurrentAction("Manipulation");
            missionPhase = 3;
        end
    elseif missionPhase == 3
        % Check if manipulation is complete
        tool_pos_error = norm(robotModel.wTt(1:3,4) - robotModel.wTg(1:3,4));
        if tool_pos_error < 0.1
            disp("Manipulation complete");
            % Optionally, end simulation or hold position
            % break;
        end        
    end

    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel, dt);

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    logger.update(sim.time, sim.loopCounter);

    % 6. Optional debug prints
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
        if missionPhase == 1
            pos_error = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));
            fprintf('Vehicle position error (m): %.3f\n', pos_error);        
        elseif missionPhase == 2
            fprintf('Heading error (rad): %.3f\n', robotModel.theta_error);
        elseif missionPhase == 3
            tool_pos_error = norm(robotModel.wTt(1:3,4) - robotModel.wTg(1:3,4));
            fprintf('Tool position error (m): %.3f\n', tool_pos_error);
        end
    end
    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);