% Plot multiple prediction horizons — 2D Unicycle version
% Called once per N value.  Each call overlays one more set of curves
% onto the persistent figure fig1.

function plotGraphMultiN( ...
    states_ts, ctrl_inputs_ts, MPC_time_ts, pfg_time_ts, ...
    reference_sig_ts, ...
    color, N, path, u_min, u_max, ...
    pathFG_max_N, sim_length, params, N_idx, N_list)

    global fig1;

    line_width = 3;
    is_first   = (N_idx == 1);

    %%
    figure(fig1);
    fig_rows = 2;
    fig_cols = 1;
    font        = 'times';
    font_size   = 20;
    font_weight = 'normal';
    set(groot, ...
        defaultAxesFontSize   = 1.0 * font_size, ...
        defaultAxesFontName   = font, ...
        defaultAxesFontWeight = font_weight, ...
        ...
        defaultLegendInterpreter     = 'latex', ...
        defaultLegendLocation        = 'best', ...
        defaultLegendBox             = 'off', ...
        defaultLegendFontSize        = 1.0 * font_size, ...
        defaultLegendFontWeight      = font_weight, ...
        defaultLegendBackgroundAlpha = 0, ...
        ...
        defaultConstantLineColor     = 'black', ...
        defaultConstantLineLineStyle = ':', ...
        defaultConstantLineLineWidth = line_width, ...
        ...
        defaultTextInterpreter = 'latex', ...
        defaultTextFontWeight  = font_weight ...
    )

    % Extract raw arrays from timeseries
    t_ctrl  = ctrl_inputs_ts.Time;           % [T × 1]
    u_data  = squeeze(ctrl_inputs_ts.Data);  % [T × n_u]  or  [n_u × T]
    t_state = states_ts.Time;                % [(T+1) × 1]
    x_data  = squeeze(states_ts.Data);       % [(T+1) × n_x]  or  [n_x × (T+1)]

    % Ensure row = sample, col = channel  (handle both orientations)
    if size(u_data,1) ~= numel(t_ctrl)
        u_data = u_data';
    end
    if size(x_data,1) ~= numel(t_state)
        x_data = x_data';
    end

    T = size(u_data, 1);   % number of control steps

    mpc_data = squeeze(MPC_time_ts.Data(:));  % [T × 1]
    pfg_data = squeeze(pfg_time_ts.Data(:));  % [T × 1]

    % Auxiliary reference from PathFG
    ref_data = squeeze(reference_sig_ts.Data);  % [T × n_x] or [n_x × T]
    if size(ref_data,1) ~= numel(reference_sig_ts.Time)
        ref_data = ref_data';
    end


    %% ---- Subplot 1: Path progress [%] ----------------------------------
    ax = subplot(fig_rows, fig_cols, 1);
    hold on;
    set(gca, 'FontSize', round(0.7*font_size));
    lgd = legend('Position',[0.70 0.73 0.1 0.05], Box='off', ...
                 BackgroundAlpha=0, FontWeight=font_weight, FontSize=0.82*font_size);
    lgd.ItemTokenSize = [20, 16];

    actual_pos = x_data(1:T, 1:2);                  % [T × 2]
    actual_cum = zeros(T, 1);
    for k = 2:T
        actual_cum(k) = actual_cum(k-1) + norm(actual_pos(k,:) - actual_pos(k-1,:));
    end
    actual_pct = 100 * actual_cum / max(actual_cum(end), eps);

    ref_pos = ref_data(1:T, 1:2);                   % [T × 2]
    ref_cum = zeros(T, 1);
    for k = 2:T
        ref_cum(k) = ref_cum(k-1) + norm(ref_pos(k,:) - ref_pos(k-1,:));
    end
    ref_pct = 100 * ref_cum / max(ref_cum(end), eps);

    if N >= pathFG_max_N
        ref_pct = 100 * ones(T,1);
    end

    plot(t_ctrl, ref_pct, Color=color, LineWidth=line_width, LineStyle=':', ...
         DisplayName=strcat('$s$ ($N{=}',num2str(N),'$)'));
    plot(t_ctrl, actual_pct, Color=color, LineWidth=line_width, ...
         DisplayName=strcat('actual ($N{=}',num2str(N),'$)'));

        hy = ylabel('Path [\%]', Interpreter='latex');
        hy.FontSize = round(1.1 * font_size); 
    xlim([0, sim_length]);


    %% ---- Subplot 2: Compute time [s] -----------------------------------
    ax = subplot(fig_rows, fig_cols, 2);
    hold on;
    set(gca, 'FontSize', round(0.7*font_size));
    lgd = legend('Position',[0.56 0.34 0.1 0.05], Box='off', ...
                 BackgroundAlpha=0, FontSize=0.8*font_size, ...
                 FontWeight=font_weight, NumColumns=2, Orientation='horizontal');
    lgd.ItemTokenSize = [20, 16];

    if N > pathFG_max_N
        compute_series = mpc_data;
        displayNameStr = strcat('Ungoverned MPC ($N{=}', num2str(N), '$)');
    else
        compute_series = pfg_data + mpc_data;
        displayNameStr = strcat('PathFG+MPC ($N{=}', num2str(N), '$)');
    end


    %% Plot in regular scale
    ylim([0, 0.22]); 
    compute_series_plot = compute_series;
    compute_series_plot(1) = compute_series_plot(2);  % align with pfg plot for visual comparison
    plot(t_ctrl, compute_series_plot, Color=color, LineWidth=line_width, DisplayName=displayNameStr);

    %% plot in log scale
    % set(gca, 'YScale','log');
    % % Decade range for this N
    % all_pos = [compute_series(:); pfg_data(:)];
    % pos_vals = all_pos(all_pos > 0);
    % if isempty(pos_vals)
    %     kmin_this = -6; kmax_this = 0;
    % else
    %     kmin_this = floor(log10(min(pos_vals)));
    %     kmax_this = ceil(log10(max(pos_vals)));
    % end

    % % First call sets limits; subsequent calls only expand
    % if is_first
    %     kmin = kmin_this;
    %     kmax = kmax_this;
    % else
    %     cur_ylim = ylim;
    %     kmin_prev = floor(log10(cur_ylim(1)));
    %     kmax_prev = round(log10(cur_ylim(2) / 3.0));
    %     kmin = min(kmin_prev, kmin_this);
    %     kmax = max(kmax_prev, kmax_this);
    % end
    % if (kmax - kmin) < 3
    %     kmin = kmax - 3;
    % end

    % % Ticks at every decade; labels only for >= 10^{-3}
    % tick_exps = kmin:kmax;
    % yticks(10.^tick_exps);
    % tick_labels = cell(size(tick_exps));
    % for ti = 1:numel(tick_exps)
    %     if tick_exps(ti) >= -3
    %         tick_labels{ti} = sprintf('10^{%d}', tick_exps(ti));
    %     else
    %         tick_labels{ti} = '';
    %     end
    % end
    % yticklabels(tick_labels);
    % ylim([10^kmin, 10^kmax * 3.0]);

    % % Plot compute series (clamp non-positive to floor for log visibility)
    % cs_plot = compute_series;
    % cs_plot(~isfinite(cs_plot) | cs_plot <= 0) = 10^kmin;
    % plot(t_ctrl, cs_plot, Color=color, LineWidth=line_width, DisplayName=displayNameStr);

    %% Plot PathFG
    if N <= pathFG_max_N
        pfg_plot = pfg_data;
        pfg_plot(1) = pfg_plot(2);  % remove first-sample spike (MATLAB artifact)
        plot(t_ctrl, pfg_plot, Color=color, LineWidth=line_width, LineStyle=':', ...
             DisplayName=strcat('PathFG ($N{=}', num2str(N), '$)'));
    end

    hy = ylabel('Compute Time [s]', Interpreter='latex');
    hy.FontSize = round(1.1 * font_size); 
    xlim([0, sim_length]);
    xlabel('Time [s]');


    %% Reorder legend entries by ascending N (on last call only)
    is_last = (N_idx == numel(N_list));
    if is_last
        [~, sort_order] = sort(N_list);
        for sp = 1:fig_rows
            subplot(fig_rows, fig_cols, sp);
            ax = gca;
            % each governed N produces 2 entries, ungoverned produces 1 (subplot 1)
            % or 1 (subplot 2); reorder children so ascending-N entries come first
            ch = ax.Children;   % plot objects in reverse-creation order
            ch = flipud(ch);    % now in creation order
            n_runs = numel(N_list);
            % figure out entries per run
            entries_per_run = zeros(n_runs, 1);
            for ri = 1:n_runs
                Ni = N_list(ri);
                if sp == 1
                    entries_per_run(ri) = 2;  % ref_pct + actual_pct
                else
                    if Ni > pathFG_max_N
                        entries_per_run(ri) = 1;  % ungoverned: compute only
                    else
                        entries_per_run(ri) = 2;  % governed: compute + pfg
                    end
                end
            end
            % build reordered index
            new_order = [];
            for ri = 1:n_runs
                si = sort_order(ri);
                offset = sum(entries_per_run(1:si-1));
                new_order = [new_order, offset+1:offset+entries_per_run(si)];
            end
            if numel(new_order) == numel(ch)
                ax.Children = flipud(ch(new_order));
            end
        end
    end

    %% Export
    fh = fig1;
    if ~isgraphics(fh), fh = gcf; end
    try
        exportgraphics(fh, 'unicyclePerformance.pdf', ...
                       'BackgroundColor','none', 'ContentType','vector');
        fprintf('Exported unicyclePerformance.pdf\n');
    catch ME
        warning(ME.identifier, '%s', ME.message);
    end

end