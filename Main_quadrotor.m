
%% TODO
% 1.PID not good: the PID of inner controller can not compute a good solution to execute
% the MPC command (Wx, Wy, Wz), resulting the nonlinear plant model produce
% a state that deviates far away from MPC predicited state. MPC expects the
% Wx, Wy, Wz command can be executed immediately and accurately, however,
% it's not the case. To make things worse, the PID in the inner controller
% sometimes produce a very large torque command that requires the 3 of
% certain propeller to be negative, which is not allowed as the motor
% cannot spin in oppositive direction. PID needs to be more conservative to
% not create nagtive thrust command for a certain propoller, also,
% thrust_min for MPC could be increase a little bit to save some negative
% space for PID to use. Or, we can replace PID with LQR for easier tuning.
% Now, all the saturation cap are removed, and the convertions from double
% to UINT are removed as well, so that the error will be caught once the
% cmd for a certain propeller becomes negative.  

%%
clear;
close all;
clc;

disp("Simulation started")


%% Add CasADi to Path
%addpath(genpath("~/Code/CasADi"));


%% Set Parameters (mass, structure...)
setCrazyflieParameters;
setNoiseParameters;


%% Specify the start and goal
start = [0.1; 0.1; 0.3];
goal = [2.5; 2.5; 1];
goal_threshold = 0.01; % unit: meter, goal reached when within the threshold


%% Obstacles
setObstacles;
obstacles = obstacles';
obstacle_sizes = obstacle_sizes';

%% Path Planner
safety_margin_universal = 0.05;
safety_margin_universal_multiple = [0.1]; % [0.1, 0.03, 0.005] Comment out if not for compare multiple paths
static_safety_margins = safety_margin_universal * ones(size(obstacle_sizes));
safety_margins_RRT = static_safety_margins + agent_size;

% valid strings: "RRT*", "Pot. Field"
path_planners = ["RRT*"];
% path_planners = ["Pot. Field"];


%% Inner Control Inclusion

inner_controller_inclusion = 0; % ATTENTION: To plot appropriate result, switch the corresponding output("to workspace" block) in the plant of simulink


%% set the lower and upper bounds of X
x_min = [-10; -10; -10; -1.0; -1.0; -1.0; -pi*0.2; -pi*0.2; -pi*0.2];
x_max = -1 * x_min; % x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot

thrust_space_saved_for_PID_inner_controller = 0.0; 
u_min = [thrust_min - mass_true * g + thrust_space_saved_for_PID_inner_controller; -0.5*pi; -0.5*pi; -0.5*pi]; % [thrust-mg,roll_dot, pitch_dot, yaw_dot]
u_max = [thrust_max - mass_true * g; 0.5*pi; 0.5*pi; 0.5*pi];


%% pathFG parameters
DSM_val_lower_threshold_pathFG = 0; % DSM_value target range lower bound
DSM_val_upper_threshold_pathFG = 0.01; % DSM_value target range upper bound
kappa_pathFG = 0.5;
iter_max_pathFG = 50; 
kappa_s = 2; % scaling factor for terminal thrust lyapunov threshold
kappa_o = 5; % the original paper use 20, scaling factor for terminal obstacle lyapunov threshold
PathFG_max_N = 30; % !!!PathFG is not included when prediction Horizon N >= 30


%% Set Frequency of Measurements and Controller
sample_time_controller_outer = 1/10; % Sample time for the outer loop controller
sample_time_pathFG = sample_time_controller_outer; % Sample time for pathFG (path Feasibility Governor)
sample_time_continous = 0.001;
sample_time_kalman_filter = 0.001; % Sample time for kalman filter


%% Linear model
% Initial conditions for the full state (p, p_dot, angles, angular_rate)
nrotor_initial_condition = [start; zeros(9, 1)];

alpha = 0; % equilibrium yaw angle
m = mass_true;

% full state continuous-time system (including outer loop and inner loop)
A_continuous =...
    [0,0,0,1,0,0,0,0,0,0,0,0;...
     0,0,0,0,1,0,0,0,0,0,0,0;...
     0,0,0,0,0,1,0,0,0,0,0,0;...
     0,0,0,0,0,0,g * sin(alpha),g * cos(alpha),0,0,0,0;...
     0,0,0,0,0,0,-g * cos(alpha),g * sin(alpha),0,0,0,0;...
     0,0,0,0,0,0,0,0,0,0,0,0;...
     0,0,0,0,0,0,0,0,0,1,0,0;...
     0,0,0,0,0,0,0,0,0,0,1,0;...
     0,0,0,0,0,0,0,0,0,0,0,1;...
     0,0,0,0,0,0,0,0,0,0,0,0;...
     0,0,0,0,0,0,0,0,0,0,0,0;...
     0,0,0,0,0,0,0,0,0,0,0,0];

B_continuous =...
    [0,0,0,0;...
     0,0,0,0;...
     0,0,0,0;...
     0,0,0,0;...
     0,0,0,0;...
     1/m,1/m,1/m,1/m;...
     0,0,0,0;...
     0,0,0,0;...
     0,0,0,0;...
     inertia_true_inverse*[layout_true(2,:);-layout_true(1,:);layout_true(3,:)]];

C_inner_and_outer = eye(12);

% outer loop continuous-time system
G = [g*sin(alpha) , g*cos(alpha), 0;...
     -g*cos(alpha), g*sin(alpha), 0;...
     0            , 0           , 0];
A_continuous_outer = [zeros(3, 3), eye(3)     , zeros(3, 3);...
                      zeros(3, 3), zeros(3, 3), G          ;...
                      zeros(3, 3), zeros(3, 3), zeros(3, 3)];
B_continuous_outer = [zeros(5, 1), zeros(5, 3);...
                      1/m        , zeros(1, 3);...
                      zeros(3, 1), eye(3)     ];

% Calculating different A/B matrices for different yaws
% m = 2.57921;  % Gazebo x500 drone
% for i = 0:11
% 
% alpha = i * pi / 6;
% 
% % outer loop continuous-time system
% G = [g*sin(alpha) , g*cos(alpha), 0;...
%      -g*cos(alpha), g*sin(alpha), 0;...
%      0            , 0           , 0];
% A_continuous_outer = [zeros(3, 3), eye(3)     , zeros(3, 3);...
%                       zeros(3, 3), zeros(3, 3), G          ;...
%                       zeros(3, 3), zeros(3, 3), zeros(3, 3)];
% B_continuous_outer = [zeros(5, 1), zeros(5, 3);...
%                       1/m        , zeros(1, 3);...
%                       zeros(3, 1), eye(3)     ];
% 
% % format long;
% [A_discrete_outer, B_discrete_outer] = c2d(A_continuous_outer, B_continuous_outer, sample_time_controller_outer);
% disp(i);
% % disp(A_discrete_outer(:,7:9));
% % disp(B_discrete_outer(:,1:3));
% disp(A_discrete_outer);
% disp(B_discrete_outer);
% 
% end
% assert(0)

C_outer = eye(9);
           
D_outer = zeros(9,4);

sys = ss(A_continuous_outer, B_continuous_outer, C_outer, D_outer);

% Discretization
% x[k+1] = A x[k] + B u[k] + Bw w[k]
% y[k] = C x[k] + D u[k] + v[k]
[A_discrete_outer, B_discrete_outer] = c2d(A_continuous_outer, B_continuous_outer, sample_time_controller_outer);
[A_discrete_outer_kalman, B_discrete_outer_kalman] = c2d(A_continuous_outer, B_continuous_outer, sample_time_kalman_filter);


%% Design MPC Controller
prediction_horizons_MPC = [15];
% prediction_horizons_MPC = [5, 15, 60, 15];

solver = 5; % read the NOTE in setMPCParameters file first

MPC_max_iters = 200; % optimization parameter

Q_MPC_diagonal = 5 * [2, 2, 2, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5]; % 5 * [2, 2, 2, 1, 1, 1, 1, 1, 1];
Q_MPC = diag(Q_MPC_diagonal);

R_MPC_diagonal = 0.1 * ones(1, 4);
R_MPC = diag(R_MPC_diagonal);


%% Design LQR (Linear Quadratic Regulator) controller

Q_LQR = Q_MPC;
R_LQR = R_MPC;

% % Continous LQR
% K_lqr_outer_loop_continous_time = lqr(sys,Q_LQR,R_LQR);

% Discrete LQR
[K_lqr_outer_loop_discrete_time, P_matrix, ~] = dlqr(A_discrete_outer, B_discrete_outer, Q_LQR, R_LQR);
K_lqr_outer_loop = K_lqr_outer_loop_discrete_time;

disp(A_discrete_outer)
disp(B_discrete_outer)
disp(P_matrix)
disp(K_lqr_outer_loop)
disp(K_lqr_outer_loop * [3.163182, -1.722803, 1.239091, 0.000000, 0.000000, 0.000000, 1.814186, 0.000000, 0.000000]')
disp((R_LQR + B_discrete_outer' * P_matrix * B_discrete_outer)^(-1) * B_discrete_outer' * P_matrix * A_discrete_outer)
% assert(0)

K = K_lqr_outer_loop;
A = A_discrete_outer;
B = B_discrete_outer;
should_be_pos_def = -K' * R_LQR * K - Q_LQR + P_matrix - (A - B * K)' * P_matrix * (A - B * K);
if all(eig(should_be_pos_def) > -1e-6)
    disp('Matrix is positive semi-definite.');
else
    disp('Matrix is not positive semi-definite');
    return;
end


%% Loop Over MPC Prediction Horizons

simulation_length = 18;
set_param('Sim_quadrotor', 'StopTime', num2str(simulation_length)); % simulation stop time
animate_traj = true;
set(groot, 'defaultFigureColor', [1, 1, 1])

if isscalar(prediction_horizons_MPC) && isscalar(safety_margin_universal_multiple)

    % create figures
    global fig1;
    % format of Position: [left, bottom, width, height]
    % fig1 = figure(Position=[0, 0, 800, 400], Name='DA FIGURE');
    fig1 = figure(Position=[0, 0, 800, 2400], Name='DA FIGURE');
    
    % loop over all the path planners
    for path_planner_idx = 1:numel(path_planners)
        path_planner = path_planners(path_planner_idx);
        pathPlanner;
        prediction_horizon_MPC = prediction_horizons_MPC(1);
        setMPCParameters;
        setStructs;

        % Simulation
        tStart = tic;
        disp("Simulink simulation started");
        clear ans;
        sim("Sim_quadrotor.slx")
        disp("Simulink simulation ended");
        tEnd = toc(tStart);
        disp("Total Simulation Time");
        disp(tEnd);

        if(solver == 5)
            temp = squeeze(ans.artificial_reference.Data);  
            artificial_reference_fixed = temp.';            
            ans.reference_signal = timeseries(artificial_reference_fixed, ans.reference_signal.Time);
        end

        % plot graph
        if isscalar(path_planners)
            plotGraphSingleN( ...
                ans.reference_signal, ...
                ans.non_linear_full_states, ...
                ans.pathFG_time, ...
                ans.MPC_time, ...
                ans.control_inputs, ...
                colors, ...
                goal, ...
                prediction_horizon_MPC, ...
                ans.aux_ref_params, ...
                path, ...
                ans.MPC_iterations, ...
                inner_controller_inclusion, ...
                mass_true, ...
                g, ...
                simulation_length, ...
                u_min, ...
                u_max, ...
                x_min, ...
                x_max);
        else
            plotGraphMultiPlanner( ...
                ans.reference_signal, ...
                ans.non_linear_full_states, ...
                ans.pathFG_time, ...
                ans.MPC_time, ...
                ans.control_inputs, ...
                colors(path_planner_idx), ...
                goal, ...
                u_min, ...
                u_max, ...
                prediction_horizon_MPC, ...
                ans.MPC_iterations, ...
                ans.aux_ref_params, ...
                ans.MPC_x0, ...
                path, ...
                path_planner, ...
                mass_true, ...
                g); 
        end

        if animate_traj
            animateTrajectory(...
                start,...
                goal,...
                obstacles,...
                obstacle_sizes,...
                rect_obs,...
                ans.non_linear_full_states,...
                ans.reference_signal,...
                agent_size, ...
                sample_time_continous,...
                sample_time_pathFG);
        end
    end
elseif isscalar(safety_margin_universal_multiple)

    path_planner = path_planners(1);% use the first path planner if there is more than one present
    pathPlanner;
    prediction_horizon_MPC = prediction_horizons_MPC(1);
    setMPCParameters;
    setStructs;

    global fig1;
    % fig1 = figure(Position=[0, 0, 800, 2000], Name='DA FIGURE');
    fig1 = figure('Position',[0, 0, 480, 800], 'Name','DA FIGURE');

    for prediction_horizon_idx = 1:numel(prediction_horizons_MPC)  
        if prediction_horizon_idx == 4   % MPCa
            solver = 5;
        else
            solver = 2;
        end       
        prediction_horizon_MPC = prediction_horizons_MPC(prediction_horizon_idx);
        setMPCParameters;
        setStructs;

        % Simulation
        disp("Simulink simulation started");
        clear ans;
        sim("Sim_quadrotor.slx");
        disp("Simulink simulation ended");

        % Plot graphs
        plotGraphMultiN( ...
            ans.reference_signal, ...
            ans.non_linear_full_states, ...
            ans.pathFG_time, ...
            ans.MPC_time, ...
            ans.control_inputs, ...
            colors(prediction_horizon_idx), ...
            goal, ...
            u_min, ...
            u_max, ...
            x_max,...
            prediction_horizon_MPC, ...
            ans.MPC_iterations, ...
            prediction_horizon_idx, ...
            ans.aux_ref_params, ...
            path, ...
            ans.MPC_x0, ...
            mass_true, ...
            g, ...
            PathFG_max_N, ...
            simulation_length);               
    end
else
    global fig1;
    fig1 = figure(Position=[0, 0, 800, 2000], Name='Multiple Paths FIGURE');
    prediction_horizon_MPC = prediction_horizons_MPC(1);
    path_planner = path_planners(1);% use the first path planner if there is more than one present

    for idx = 1:numel(safety_margin_universal_multiple)
        safety_margins_RRT = safety_margin_universal_multiple(idx) * ones(size(obstacle_sizes)) + agent_size;
        pathPlanner;
        prediction_horizon_MPC = prediction_horizons_MPC(1);
        setMPCParameters;
        setStructs;

        setMPCParameters;
        setStructs;

        % Simulation
        disp("Simulink simulation started");
        clear ans;
        sim("Sim_quadrotor.slx");
        disp("Simulink simulation ended");

        % Plot graphs
        plotGraphMultiPath( ...
            ans.reference_signal, ...
            ans.non_linear_full_states, ...
            ans.pathFG_time, ...
            ans.MPC_time, ...
            ans.control_inputs, ...
            colors(idx), ...
            goal, ...
            u_min, ...
            u_max, ...
            x_max,...
            safety_margin_universal_multiple(idx), ...
            ans.MPC_iterations, ...
            idx, ...
            ans.aux_ref_params, ...
            path, ...
            ans.MPC_x0, ...
            mass_true, ...
            g, ...
            PathFG_max_N, ...
            simulation_length);      
    end

end


%% Plot multiple 3D trajectories for comparison
% global fig1;
% fig1 = figure('Position',[0 0 1200 800],'Name','Multiple Paths FIGURE');
% ax1 = axes(fig1);
% hold(ax1,'on'); 
% grid(ax1,'on'); 
% axis(ax1,'equal');
% xlim(ax1,[0 3]); ylim(ax1,[0 3]); zlim(ax1,[0 2]);
% xlabel(ax1,'$x$ [m]', 'FontSize', 15); 
% ylabel(ax1,'$y$ [m]', 'FontSize', 15);  
% zlabel(ax1,'$z$ [m]', 'FontSize', 15); 
% ax1.FontSize = 14;  % increases axis tick labels
% view(ax1, -7.573304666576624,19.585200472715634);
% % campos(ax1, [-1.412012197832373,-20.40253712848401,8.86123732595924]);
% % view(0,90); % first param is to rotate along z-axis.
% 
% prediction_horizon_MPC = prediction_horizons_MPC(1);
% 
% % ---------------- 1st Simulation ----------------
% path_planner = path_planners(1); % or "Pot. Field"
% pathPlanner;
% solver = 2;
% setMPCParameters;
% setStructs;
% clear ans;
% sim("Sim_quadrotor.slx");
% animateTrajectoryMultiSolver(...
%     start, goal, obstacles, obstacle_sizes,rect_obs, ...
%     ans.non_linear_full_states, ans.reference_signal.Data, ...
%     agent_size, sample_time_continous, sample_time_pathFG, ...
%     ax1, 'b', 'g', 'Actual Traj. (RRT*+PathFG+MPC)', 'Aux. Ref. (RRT*+PathFG+MPC)');  % 'b'=trajectory color, 'r'=reference color
% 
% 
% % ---------------- 2rd Simulation ----------------
% path_planner = "Pot. Field"; % or "Pot. Field"
% pathPlanner;
% solver = 2;
% setMPCParameters;
% setStructs;
% clear ans;
% sim("Sim_quadrotor.slx");
% animateTrajectoryMultiSolver(...
%     start, goal, obstacles, obstacle_sizes, rect_obs,...
%     ans.non_linear_full_states, ans.reference_signal.Data, ...
%     agent_size, sample_time_continous, sample_time_pathFG, ...
%     ax1, [0.5 0.5 0.5], [1.0, 0.8, 0.0], 'Actual Traj. (Pot.Field+PathFG+MPC)', 'Aux. Ref. (Pot.Field+PathFG+MPC)');  % 'g'=trajectory color, 'm'=reference color
% 
% 
% % ---------------- 3rd Simulation ----------------
% path_planner = "RRT*"; % or "Pot. Field"
% pathPlanner;
% solver = 5;
% setMPCParameters;
% setStructs;
% clear ans;
% sim("Sim_quadrotor.slx");
% temp = squeeze(ans.artificial_reference.Data);  
% artificial_reference_fixed = temp.';
% animateTrajectoryMultiSolver(...
%     start, goal, obstacles, obstacle_sizes, rect_obs,...
%     ans.non_linear_full_states, artificial_reference_fixed, ...
%     agent_size, sample_time_continous, sample_time_pathFG, ...
%     ax1, [0.776, 0.294, 0.549], 'm', 'Actual Traj. (MPCa)', 'Aux. Ref. (MPCa)');  % 'g'=trajectory color, 'm'=reference color
% 
% 
% %% ---------------- Legend ----------------
% % Only show one entry per trajectory type (reference/actual) for clarity
% hLines = findobj(ax1, 'Type','line'); 
% hLines = flip(hLines);  % now first plotted lines come first
% [~, idx] = unique({hLines.DisplayName}, 'stable'); 
% lgd = legend(ax1, hLines(idx), 'Position', [0.6 0.73 0.2 0.15]);  % top right
% lgd.Box = 'off';  % removes the black box border
% lgd.FontSize = 15;

% %% Draw start and goal with legend
% plot3(ax1, start(1), start(2), start(3), 'go', 'MarkerSize',10, 'DisplayName','Start');
% plot3(ax1, goal(1), goal(2), goal(3), 'bo', 'MarkerSize',10, 'DisplayName','Goal');

% exportgraphics(fig1, '3dTraj.pdf', 'ContentType', 'vector');
exportgraphics(gcf, '3dTraj.png', 'Resolution', 600);  


disp("Simulation ended");


%% Appendix

% full state continuous-time system (including outer loop and inner loop)
% A_continouse_inner_and_outer = [0,0,0,1,0,0,0,0,0,0,0,0;...
%      0,0,0,0,1,0,0,0,0,0,0,0;...
%      0,0,0,0,0,1,0,0,0,0,0,0;...
%      0,0,0,0,0,0,g * sin(alpha),g * cos(alpha),0,0,0,0;...
%      0,0,0,0,0,0,-g * cos(alpha),g * sin(alpha),0,0,0,0;...
%      0,0,0,0,0,0,0,0,0,0,0,0;...
%      0,0,0,0,0,0,0,0,0,1,0,0;...
%      0,0,0,0,0,0,0,0,0,0,1,0;...
%      0,0,0,0,0,0,0,0,0,0,0,1;...
%      0,0,0,0,0,0,0,0,0,0,0,0;...
%      0,0,0,0,0,0,0,0,0,0,0,0;...
%      0,0,0,0,0,0,0,0,0,0,0,0];

% B_countinous_inner_and_outer = [0,0,0,0;...
%      0,0,0,0;...
%      0,0,0,0;...
%      0,0,0,0;...
%      0,0,0,0;...
%      1/mass_for_controller,0,0,0;...
%      0,1,0,0;...
%      0,0,1,0;...
%      0,0,0,1;
%      0,1/inertia_for_controller(1,1),0,0;
%      0,0,1/inertia_for_controller(2,2),0;
%      0,0,0,1/inertia_for_controller(3,3)];

% C_inner_and_outer =  [1,0,0,0,0,0,0,0,0,0,0,0;...
%                       0,1,0,0,0,0,0,0,0,0,0,0;...
%                       0,0,1,0,0,0,0,0,0,0,0,0;...
%                       0,0,0,0,0,0,1,0,0,0,0,0;...
%                       0,0,0,0,0,0,0,1,0,0,0,0;...
%                       0,0,0,0,0,0,0,0,1,0,0,0;...
%                       0,0,0,0,0,0,0,0,0,1,0,0;...
%                       0,0,0,0,0,0,0,0,0,0,1,0;...
%                       0,0,0,0,0,0,0,0,0,0,0,1];



% %% Kalman Filter Design
% % Covariances
% % Rv = measurement_noise_full_state_covariance_matrix([1,2,3,7,8,9],[1,2,3,7,8,9]);
% Rv = measurement_noise_full_state_covariance_matrix(1:9, 1:9);
% disturbance_covariance = 0.001 * eye(9);
% % test
% % error: The block could not compute a convergent Kalman estimator for this
% % plant model and covariance data. To remedy the problem:
% % 1. Make sure that all unstable poles of A are observable through C (use MINREAL to check)
% % 2. Modify the weights Q, R and N to make [G 0;H I]*[Q N;N' R]*[G 0;H I]' positive
% % definite (use EIG to check).
% disp("eigenvalues of [G 0;H I]*[Q N;N' R]*[G 0;H I]':");
% matrix1 = [eye(9), zeros(9,6); zeros(6,9), eye(6,6)];
% matrix2 = [zeros(9), zeros(9,6); zeros(6,9), Rv];
% disp(eig(matrix1 * matrix2 * matrix1'));