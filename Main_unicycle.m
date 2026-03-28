%%  Main_unicycle.m — 2D Unicycle Path Planning & Simulation
%   Entry point for the unicycle workspace.
%   Phase 1: parameter setup + RRT* path planning + visualization.
%   Phase 2 (TODO): MPC tracking via Simulink.
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


%% ---- Set parameters ----------------------------------------------------
setSysParams;       % start, goal, state_min/max, input_min/max
setObstacles;       % rect_obs, buffer

% setPlannerParams;        
% setPathfgParams;    
% computeAndPack;    
% setMPCParams;
% setStructs;



%% ---- Run path planner --------------------------------------------------
disp("Running RRT* path planner ...");
RRT2;  % populates: path, size_of_path, rrt_comp_time

disp("Path planning complete.");
disp(strcat("  Waypoints: ", num2str(size_of_path)));


%% ---- TODO: MPC + Simulink integration ---------------------------------

% prediction_horizon_MPC = N;
% sim_start = tic;
% disp("Simulink simulation started");
% sim_out = sim("sim.slx");
% disp("Simulink simulation ended");
% sim_time = toc(sim_start);
% disp(strcat("Total simulation time: ", num2str(sim_time)));

disp("=== Unicycle Simulation Ended ===");