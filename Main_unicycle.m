%%  Main_unicycle.m — 2D Unicycle Path Planning & Simulation
%   Entry point for the unicycle workspace.
clear;
close all;
clc;

disp("=== Unicycle Simulation Started ===");

%% ---- Add paths --------------------------------------------------------
addpath('params');
addpath('pathPlanning');
addpath('plot');
addpath('plant');
addpath('controller');
addpath('governor');
addpath('dynamicSafetyMargin');
addpath('collisionCheck');


%% ---- Set parameters ----------------------------------------------------
setSysParams;
setObstacles;
setPathfgParams;
setMPCParams; 

% set frequencies
sample_time_pathFG = 0.1;
controller_sampling_time = 0.1;
sample_time_controller_outer = 0.1;
sample_time_continous = 0.1;

%% ---- Run path planner --------------------------------------------------
disp("Running RRT* path planner ...");
RRT2;  % populates: path, size_of_path, rrt_comp_time

disp("Path planning complete.");
disp(strcat("  Waypoints: ", num2str(size_of_path)));


%% ---- plot ----
global fig1;                                                    
fig1 = figure('Position',[0, 0, 960, 550], 'Name','Multi-N Comparison');

% Map each N to its color by sorted rank (so legend order = ascending N)
[~, sorted_rank] = sort(N_list);
disp("sorted_rank:");
disp(sorted_rank);
color_idx = zeros(size(N_list));
color_idx(sorted_rank) = 1:numel(N_list);

% Storage for multi-solver overlay plot
sim_results = cell(numel(N_list), 1);

for N_idx = 1:numel(N_list)
    N = N_list(N_idx);
    fprintf('\n====== Run %d / %d  (N = %d) ======\n', N_idx, numel(N_list), N);

    % Re-pack params with the current N
    computeAndPack;

    % Simulation
    disp("Simulink simulation started");
    sim("Sim_unicycle.slx");
    disp("Simulink simulation ended");

    % Store results for the combined plot later
    sim_results{N_idx}.states    = ans.states;
    sim_results{N_idx}.ctrl      = ans.ctrl_inputs;
    sim_results{N_idx}.ref       = ans.reference_signal;
    sim_results{N_idx}.N         = N;
    sim_results{N_idx}.color_idx = color_idx(N_idx);

    % Plot graphs (per-N performance subplots)
    plotGraphMultiN( ...
        ans.states, ...
        ans.ctrl_inputs, ...
        ans.MPC_time, ...
        ans.pathFG_time, ...
        ans.reference_signal, ...
        colors(color_idx(N_idx)), ...
        N, ...
        path, ...
        input_min, ...
        input_max, ...
        pathFG_max_N, ...
        sim_duration, ...
        params, ...
        N_idx, ...
        N_list);
end


%% Plot multiple 2D trajectories for comparison
plotMultiN2D(sim_results, rect_obs, buffer, start, goal, ...
             colors, color_idx, pathFG_max_N, N_list);


disp("=== Unicycle Simulation Ended ===");