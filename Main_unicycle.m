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


%% Plot multiple 2D trajectories for comparison (like quadrotor Main)
fig_multi = figure('Position',[0 50 1200 500], 'Name','Multi-N 2D Trajectories');
ax_multi = axes(fig_multi);
hold(ax_multi,'on'); grid(ax_multi,'on'); axis(ax_multi,'equal');
xlabel(ax_multi,'$x$ [m]', 'FontSize',15, 'Interpreter','latex');
ylabel(ax_multi,'$y$ [m]', 'FontSize',15, 'Interpreter','latex');
xlim(ax_multi, [-1,9]);
ylim(ax_multi, [-0.5,6.5]);
ax_multi.FontSize = 14;

% Draw obstacles (very light blue, publication-friendly)
obs_color = [0.85 0.92 1.0];
n_obs = size(rect_obs, 1);
for kk = 1:n_obs
    x1 = rect_obs(kk,1); y1 = rect_obs(kk,2);
    x2 = rect_obs(kk,3); y2 = rect_obs(kk,4);
    fill(ax_multi, [x1 x2 x2 x1], [y1 y1 y2 y2], ...
         obs_color, 'EdgeColor','none', ...
         'HandleVisibility','off');
end
% % Inflated obstacle outlines (red dashed)
% for kk = 1:n_obs
%     bx1 = rect_obs(kk,1)-buffer; by1 = rect_obs(kk,2)-buffer;
%     bx2 = rect_obs(kk,3)+buffer; by2 = rect_obs(kk,4)+buffer;
%     plot(ax_multi, [bx1 bx2 bx2 bx1 bx1], [by1 by1 by2 by2 by1], ...
%          'r--', 'LineWidth', 0.8, 'HandleVisibility','off');
% end

% Draw start & goal
plot(ax_multi, start(1), start(2), 'go', 'MarkerSize',10, ...
     'DisplayName','Start');
plot(ax_multi, goal(1),  goal(2),  'bo', 'MarkerSize',10, ...
    'DisplayName','Goal');

% Overlay each simulation (sorted by N for consistent legend order)
[~, sorted_order] = sort(cellfun(@(s) s.N, sim_results));

% Trajectory / reference colour pairs (matching quadrotor style)
% traj_colors = {'b', [0.5 0.5 0.5], [0.776 0.294 0.549]};
traj_colors = {colors(color_idx(2)), colors(color_idx(3)), colors(color_idx(1))};
ref_colors  = {'g', [1.0 0.8 0.0], 'm'};

colors(color_idx(N_idx)), ...

for ii = 1:numel(sorted_order)
    si = sorted_order(ii);
    s  = sim_results{si};
    tc = traj_colors{min(ii, numel(traj_colors))};
    rc = ref_colors{min(ii, numel(ref_colors))};
    traj_name = sprintf('actual (N=%d)', s.N);
    ref_name  = sprintf('Aux. Ref. (N=%d)', s.N);

    animateTrajectoryMultiSolver(...
        start, goal, rect_obs, ...
        s.states, s.ref, ...
        buffer, ...
        ax_multi, tc, rc, traj_name, ref_name);
end

% Legend — deduplicate, one entry per trajectory type (like quadrotor Main)
hLines = findobj(ax_multi, 'Type','line');
hLines = flip(hLines);
[~, uid] = unique({hLines.DisplayName}, 'stable');
lgd = legend(ax_multi, hLines(uid), 'Position', [0.58 0.67 0.2 0.15]);
lgd.Box = 'off';
lgd.FontSize = 15;

exportgraphics(fig_multi, 'unicycleNavigation.png', 'Resolution', 600);


disp("=== Unicycle Simulation Ended ===");