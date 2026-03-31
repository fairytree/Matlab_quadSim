function plotMultiN2D(sim_results, rect_obs, buffer, start, goal, ...
                      colors, color_idx, pathFG_max_N, N_list)
% plotMultiN2D  Multi-N 2D trajectory overlay plot (publication quality).
%
%   Creates a horizontal figure, draws obstacles, overlays each solver's
%   actual trajectory and (for governed cases) the auxiliary reference,
%   adds start/goal markers, legend, and exports to PDF.
%
%   Ungoverned MPC (N > pathFG_max_N) is automatically detected:
%     - labelled with N* instead of N
%     - auxiliary reference line is suppressed
%
%   INPUTS
%     sim_results   {nRuns×1} cell   each entry is a struct with fields:
%                     .states   (timeseries), .ref (timeseries or matrix),
%                     .N (scalar), .color_idx (scalar)
%     rect_obs      [n_obs × 4]      rectangular obstacles [x1 y1 x2 y2]
%     buffer        scalar            safety inflation radius
%     start, goal   [2×1]             start / goal positions
%     colors        color map array   indexed via color_idx
%     color_idx     [nRuns×1]         colour index per run (sorted rank)
%     pathFG_max_N  scalar            max N for governed PathFG
%     N_list        [1×nRuns]         list of N values (original order)

    set(groot, 'defaultTextInterpreter', 'latex');

    %% ---- Figure & axes -------------------------------------------------
    fig_multi = figure('Position',[0 50 1200 500], 'Name','Multi-N 2D Trajectories');
    ax = axes(fig_multi);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    xlabel(ax,'$x$ [m]', 'FontSize',15, 'Interpreter','latex');
    ylabel(ax,'$y$ [m]', 'FontSize',15, 'Interpreter','latex');
    xlim(ax, [-1, 9.5]);
    ylim(ax, [-0.5, 6.5]);
    ax.FontSize = 14;

    %% ---- Obstacles (very light blue) -----------------------------------
    obs_color = [0.85 0.92 1.0];
    n_obs = size(rect_obs, 1);
    for kk = 1:n_obs
        x1 = rect_obs(kk,1); y1 = rect_obs(kk,2);
        x2 = rect_obs(kk,3); y2 = rect_obs(kk,4);
        fill(ax, [x1 x2 x2 x1], [y1 y1 y2 y2], ...
             obs_color, 'EdgeColor','none', 'HandleVisibility','off');
    end

    %% ---- Overlay each simulation (sorted by ascending N) ---------------
    [~, sorted_order] = sort(cellfun(@(s) s.N, sim_results));

    % Build per-rank colour lists (same order as sorted_order)
    n_runs = numel(sorted_order);
    traj_colors = cell(1, n_runs);
    ref_colors  = cell(1, n_runs);
    for ii = 1:n_runs
        si = sorted_order(ii);
        traj_colors{ii} = colors(sim_results{si}.color_idx);
        ref_colors{ii}  = colors(sim_results{si}.color_idx);
    end

    for ii = 1:n_runs
        si = sorted_order(ii);
        s  = sim_results{si};
        tc = traj_colors{ii};
        rc = ref_colors{ii};

        % Auto-detect ungoverned MPC
        is_ungoverned = (s.N > pathFG_max_N);

        if is_ungoverned
            traj_name = sprintf('Actual Traj.($N^*{=}%d$)', s.N);
            ref_name  = '';     % suppressed
        else
            traj_name = sprintf('Actual Traj.($N{=}%d$)', s.N);
            ref_name  = sprintf('Aux. Ref.($N{=}%d$)', s.N);
        end

        animateTrajectoryMultiSolver(...
            start, goal, rect_obs, ...
            s.states, s.ref, ...
            buffer, is_ungoverned, ...
            ax, tc, rc, traj_name, ref_name);
    end

    %% ---- Start & Goal --------------------------------------------------
    plot(ax, start(1), start(2), 'go', 'MarkerSize',10, 'DisplayName','Start');
    plot(ax, goal(1),  goal(2),  'bo', 'MarkerSize',10, 'DisplayName','Goal');

    %% ---- Legend (deduplicated, shortened reference line) ----------------
    hLines = findobj(ax, 'Type','line');
    hLines = flip(hLines);
    [~, uid] = unique({hLines.DisplayName}, 'stable');
    lgd = legend(ax, hLines(uid), 'Position', [0.58 0.65 0.2 0.15]);
    lgd.Box = 'off';
    lgd.FontSize = 15;
    lgd.ItemTokenSize = [20, 12];   % 2/3 of default [30,18]

    %% ---- Export ---------------------------------------------------------
    exportgraphics(fig_multi, 'unicycleNavigation.png', 'Resolution', 600); 

    % exportgraphics(fig_multi, 'unicycleNavigation.pdf', 'ContentType','vector');

    % fh = fig_multi;
    % if ~isgraphics(fh), fh = gcf; end
    % try
    %     exportgraphics(fh, 'unicycleNavigation.pdf', ...
    %                    'BackgroundColor','none', 'ContentType','vector');
    %     fprintf('Exported unicycleNavigation.pdf\n');
    % catch ME
    %     warning(ME.identifier, '%s', ME.message);
    % end
end
