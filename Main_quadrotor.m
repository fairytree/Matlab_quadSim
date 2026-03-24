
clear;
close all;
clc;

disp("Begin of Quadrotor Simulation")

%% Specify the start and goal
start = [0.1; 0.1; 0.3];
goal = [2.5; 2.5; 1.0]; % need to adjust bounds_rrt for different goal
goal_threshold = 0.01;  % unit: meter


%% Set Parameters (mass, structure...)
setCrazyflieParameters;
setNoiseParameters;
init_flag = false; % To remove the initial computation spike caused by Simulink, preventing it from polluting the measured computation time. 


%% choose Reference Source, Controller, State Estimator and Path Planning Algorithm
ref_source_list = [2, 3]; % ERG = 1;  FG = 2 (FG3); pathFG = 3; sine wave signal = 4; step signal = 5;  (Be careful when change the index, MPC computes based on the ref source)
controller = 2; % LQR = 1, MPC = 2; 
path_planner = 2; % navigation field = 1; RRT* = 2; % Note!!! navigation field is not updated for the new plotting. 
state_estimator = 3; % kalman filter = 1 (not working); observer = 2; non-linear true states = 3; built in Kalman Filter = 4; 
DSM_inclusion = 1;
pathFG_inclusion = 1;
navigation_field_repulsion_inclusion = 1;
rrt_star_inclusion = 1; % use RRT*


%% Obstacles
setObstacles;
obstacles = obstacles';
obstacle_sizes = obstacle_sizes';
safety_margin_universal = 0.03; % the same safety margin for all the obstacles, for collision check


%% ERG parameters
influence_margins =  0.2 * ones(size(obstacle_sizes)); 
% NOTE!!!: when safety_margin_universal is very small (such as 0.01), static_saftey_margins
% has to be much larger than safety_margin_universal (such as 3.4X) for FG3 or make the smoothing_radius 
% really small around 0.01 (v_dot go at full speed of unit vector until reach the smoothing_radius
% range), otherwise it cannot pass; however, PathFG only needs to be slightly bigger (such as 1.2X).
static_saftey_margins = 1.0 * safety_margin_universal * ones(size(obstacle_sizes)); % static_saftey_margins has to be smaller than influence_margins
circulation_gains = 0.1 * ones(size(obstacle_sizes)); 

smoothing_radius = 0.1; % unit: meter, if smoothing_radius is small, the agent goes at full speed until super close to obstacles
step_size = 0.1;
max_iterations = 10000;
epsilon = 0.5; % epsilon is between 0 and 1
k_P = 13; % paper uses 13
k_D = 5; % paper uses 5


%% FG parameters
max_iter_FG_statics_check = 50; 
kappa_FG = 1;
kappa_floor_FG = 0.2;
kappa_ceiling_FG = 1.0;
lambda_FG = 0.8;
delta_FG = 0.05;
% NOTE!!! prediction_horizon_FG is M-N steps according to the original paper 
prediction_horizon_FG = 0; % not in use, so that FG3 and PathFG have the same terminal set. 
scaling_factor_traj_DSM_obstacle = 20; % scaling factor for Y boundaries (including states, control inputs and obstacle boundaries)
scaling_factor_traj_DSM_x_bounds = 3;
scaling_factor_traj_DSM_U_bounds = 3;


%% pathFG parameters
DSM_val_lower_threshold_pathFG = 0; % DSM_value target range lower bound
DSM_val_upper_threshold_pathFG = 0.5; % DSM_value target range upper bound
kappa_pathFG = 0.5;
iter_max_pathFG = 10;
PathFG_max_N = 30; % !!!PathFG is not included when prediction Horizon N >= 30


%% RRT parameters
tree_color = "b";
safety_margins_RRT = static_saftey_margins + agent_size;
goal_frequency_rrt = 0.5;
max_distance_rrt_star = 1;
animate_rrt = 0;
max_iterations_rrt = 10000;
step_size_rrt = 0.1;
threshold_rrt = 0.1;
tree_rrt = [start', 0, 0, norm(goal - start)];
plot_nodes = false;
% NOTE!!! the number of ref_reached_threshold_rrt needs to be carefully
% tunned. If too big, the ref is switched to the next waypoint of RRT* before reaching
% the prior waypoint, maybe result in invalid path (the ray connecting current
% state and next waypoint collides with the obstacles) and thereby extremely small DSM_val
% for the current state, and the agent gets stuck. On the other hand, if too small,
% the v_dot is very small for many steps because it tried to get close
% enough to the current reference waypoint, which may also in turn results
% in agent getting stuck. 
ref_reached_threshold_rrt = 0.03; 
optimal_path = start'; % NOTE!!! The path for PathFG is from start to goal, while other governors use path that's from goal to start. 
size_of_optimal_path = 1;
if(path_planner == 2)
    RRT2;
end

%% set the lower and upper bounds of X
x_min = [-10; -10; -10; -1.0; -1.0; -1.0; -pi*0.2; -pi*0.2; -pi*0.2];
x_max = -1 * x_min; % x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot

thrust_space_saved_for_PID_inner_controller = 0.0; 
u_min = [thrust_min - mass_true * g + thrust_space_saved_for_PID_inner_controller; -0.5*pi; -0.5*pi; -0.5*pi]; % [thrust-mg,roll_dot, pitch_dot, yaw_dot]
u_max = [thrust_max - mass_true * g; 0.5*pi; 0.5*pi; 0.5*pi];

%% Specify which noise signals noise to include
measurement_noise_full_state_inclusion_multiplier = 0;
measurement_noise_body_rates_inclusion_multiplier = 0; % TODO, make sure the noise generation block is correct in simulink
measurement_noise_body_accelerations_inclusion_multiplier = 0; % TODO, make sure the noise generation block is correct in simulink


%% Set Frequency of Measurements and Controller
sample_time_measurements_full_state = 1/200;
sample_time_measurements_body_rates = 1/500;
sample_time_measurements_body_accelerations = 1/500;
sample_time_controller_outer = 1/10;
sample_time_controller_inner = 1/500;
sample_time_kalman_filter = 1/1000;
sample_time_FG = sample_time_controller_outer;
sample_time_pathFG = sample_time_controller_outer;
sample_time_continous = 1/1000;


%% Linear model

% Initial conditions for the full state
nrotor_initial_condition = [...
        start(1,1); start(2,1); start(3,1);...
        0; 0; 0;...
        0; 0; 0;...
        0; 0; 0;...
        ];

alpha = 0; % equilibrium yaw angle

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

C_inner_and_outer = eye(12);

% outer loop continuous-time system
A_continuous_outer = [...
                0,0,0,  1,0,0,  0,0,0   ;...
                0,0,0,  0,1,0,  0,0,0   ;...
                0,0,0,  0,0,1,  0,0,0   ;...
                                      ...
                0,0,0,  0,0,0,  g*sin(alpha),g*cos(alpha),0   ;...
                0,0,0,  0,0,0,  -g*cos(alpha),g*sin(alpha),0   ;...
                0,0,0,  0,0,0,  0,0,0   ;...
                                         ...
                0,0,0,  0,0,0,  0,0,0   ;...
                0,0,0,  0,0,0,  0,0,0   ;...
                0,0,0,  0,0,0,  0,0,0   ;...
                ];

B_continuous_outer = [...
                0,0,0,0;...
                0,0,0,0;...
                0,0,0,0;...
                0,0,0,0;...
                0,0,0,0;...
                1/mass_for_controller,0,0,0;...
                0,1,0,0;...
                0,0,1,0;...
                0,0,0,1;...
                ];

% C_outer = [1,0,0,0,0,0,0,0,0;...
%            0,1,0,0,0,0,0,0,0;...
%            0,0,1,0,0,0,0,0,0;...
%            0,0,0,0,0,0,1,0,0;...
%            0,0,0,0,0,0,0,1,0;...
%            0,0,0,0,0,0,0,0,1;...
%            ];...

C_outer = eye(9);
           
% D_outer = zeros(6,4);
D_outer = zeros(9,4);

sys = ss(A_continuous_outer,B_continuous_outer,C_outer,D_outer);

% Discretization
% x[k+1] = A x[k] + B u[k] + Bw w[k]
% y[k] = C x[k] + D u[k] + v[k]
[A_discrete_outer, B_discrete_outer] = c2d(A_continuous_outer, B_continuous_outer, sample_time_controller_outer);
[A_discrete_outer_kalman, B_discrete_outer_kalman] = c2d(A_continuous_outer, B_continuous_outer, sample_time_kalman_filter);


%% Observer Design (Not working)
polevec_observer = 6 * [-100+3j,-100-3j,-120+2j,-120-2j,-110,-115,-140,-145,-150];
L = (place(A_continuous_outer', C_outer', polevec_observer))';
% obsv_continous = obsv(A_continouse_inner_and_outer, C_inner_and_outer);
% rank_obsv_continous = rank(obsv_continous);
% obsv_discrete = obsv(A_discrete_outer_kalman, C_outer);
% rank_obsv_discrete = rank(obsv_discrete);


%% Kalman Filter Design (Not working)
% Covariances
% Rv = measurement_noise_full_state_covariance_matrix([1,2,3,7,8,9],[1,2,3,7,8,9]);
Rv = measurement_noise_full_state_covariance_matrix(1:9, 1:9);
disturbance_covariance = 0.001 * eye(9);


%% Design MPC Controller
prediction_horizon_MPC = 5;
MPC_max_iters = 50; % optimization parameter
Q_MPC_diagonal = 5 * [2, 2, 2, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5]; % 5 * [2, 2, 2, 1, 1, 1, 1, 1, 1];
Q_MPC = diag(Q_MPC_diagonal);
R_MPC_diagonal = 0.1 * ones(1, 4);
R_MPC = diag(R_MPC_diagonal);


%% Design LQR (Linear Quadratic Regulator) controller
Q_LQR = Q_MPC;
R_LQR = R_MPC;

% Continous LQR
K_lqr_outer_loop_continous_time = lqr(sys,Q_LQR,R_LQR);

% Discrete LQR
[K_lqr_outer_loop_discrete_time, LQR_P_matrix_full, ~] = dlqr(A_discrete_outer, B_discrete_outer, Q_LQR, R_LQR);
K_lqr_outer_loop = K_lqr_outer_loop_discrete_time;

setMPCParameters;
setStructs;


%% Start Simulation
simulation_length = 30;
set_param('Sim_quadrotor', 'StopTime', num2str(simulation_length)); % Set the simulation stop time for the model
animate_traj = true;
set(groot, 'defaultFigureColor', [1, 1, 1])

simOut(numel(ref_source_list)) = Simulink.SimulationOutput;

% plot
global fig1;
fig1 = figure(Position=[0, 0, 480, 2200], Name='DA FIGURE');

for idx = 1:numel(ref_source_list)
    ref_source = ref_source_list(idx);
    disp("ref_source");
    disp(ref_source);

    if(ref_source == 3) % PathFG
        kappa_s = 6; 
        kappa_o = 20; 
        kappa_omega = 6;
    else
        kappa_s = 100; % try 240, Note!!!  the original paper (invariant set distributed) use 6, scaling factor for terminal thrust lyapunov threshold
        kappa_o = 200; % try 800 Note!!! the original paper use 20, scaling factor for terminal obstacle lyapunov threshold
        kappa_omega = 100; % try 240 Note!!! the original paper doesn't have this term
    end

    setMPCParameters;
    setStructs;

    % Simulation
    disp("Simulink simulation started");
    simOut(idx) = sim("Sim_quadrotor.slx");
    disp("Simulink simulation ended");
              
end

% Plot graphs
for idx = numel(ref_source_list):-1:1
    ref_source = ref_source_list(idx);
    if(ref_source == 2)
        Ncycles = length(simOut(idx).governor_time.Data);
        Nfast   = floor(length(simOut(idx).path_planner_time.Data) / Ncycles);
        governer_total_time = ...
            sum(reshape(simOut(idx).path_planner_time.Data(1:Nfast*Ncycles), Nfast, []),1).' + ...
            sum(reshape(simOut(idx).DSM_time.Data(1:Nfast*Ncycles),          Nfast, []),1).' + ...
            simOut(idx).governor_time.Data + ...
            simOut(idx).dynamic_check_time.Data;
    else
        governer_total_time = simOut(idx).path_planner_time.Data + simOut(idx).DSM_time.Data;
    end
    
    plotGraphMultiGovernor( ...
        simOut(idx).reference_signal, ...
        simOut(idx).non_linear_full_states, ...
        governer_total_time, ...
        simOut(idx).controller_time, ...
        simOut(idx).control_inputs, ...
        colors, ...
        goal, ...
        prediction_horizon_MPC, ...
        simOut(idx).aux_ref_params, ...
        optimal_path, ...
        mass_true, ...
        g, ...
        simulation_length, ...
        u_min, ...
        u_max, ...
        x_min, ...
        x_max, ...
        ref_source);   
end


%% Animate Trajectory
% animateTrajectory(...
%     start,...
%     goal,...
%     obstacles,...
%     obstacle_sizes,...
%     simOut(idx).non_linear_full_states,...
%     simOut(idx).reference_signal,...
%     agent_size, ...
%     sample_time_continous,...
%     sample_time_pathFG)

disp("End of Quadrotor Simulation");