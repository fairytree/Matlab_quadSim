% Plot multiple prediction horizons — 2D Unicycle version
% Adapted from the quadrotor plotGraphMultiN.m, same visual format.
%
% Called once per N value.  Each call overlays one more set of curves
% onto the persistent figure fig1.
%
% INPUTS  (all Simulink timeseries from "To Workspace" blocks)
%   states_ts         timeseries   actual system states     [T × n_x]
%   ctrl_inputs_ts    timeseries   MPC control inputs       [T × n_u]
%   MPC_time_ts       timeseries   MPC solve time           [T × 1]
%   pfg_time_ts       timeseries   PathFG solve time        [T × 1]
%   reference_sig_ts  timeseries   PathFG auxiliary ref     [T × n_x]
%   color             color spec   line colour for this run
%   N                 scalar       prediction horizon
%   path              [W × 2]     RRT* waypoints
%   u_min             [n_u × 1]   input lower bounds
%   u_max             [n_u × 1]   input upper bounds
%   pathFG_max_N      scalar       max N where PathFG is used
%   sim_length        scalar       total simulation time (s)
%   params            struct       packed parameters

function plotGraphMultiN( ...
    states_ts, ctrl_inputs_ts, MPC_time_ts, pfg_time_ts, ...
    reference_sig_ts, ...
    color, N, path, u_min, u_max, ...
    pathFG_max_N, sim_length, params)

    global fig1;

    fig_count  = 1;
    line_width = 2;

    %%

    figure(fig1);
    fig_rows = 4;
    fig_cols = 1;
    font        = 'times';
    font_size   = 12;
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
    lgd = legend('Position',[0.74 0.837 0.1 0.05], Box='off', ...
                 BackgroundAlpha=0, FontWeight=font_weight, FontSize=0.82*font_size);
    lgd.ItemTokenSize = [12, 10];

    % total path length (RRT* path)
    path_length = 0;
    for node_idx = 1:size(path,1)-1
        path_length = path_length + norm(path(node_idx+1,:) - path(node_idx,:));
    end

    % actual state cumulative distance  (normalised by path length)
    actual_pos = x_data(1:T, 1:2);                  % [T × 2]
    actual_cum = zeros(T, 1);
    for k = 2:T
        actual_cum(k) = actual_cum(k-1) + norm(actual_pos(k,:) - actual_pos(k-1,:));
    end
    actual_pct = 100 * actual_cum / max(path_length, eps);

    % auxiliary reference cumulative distance  (normalised by path length)
    ref_pos = ref_data(1:T, 1:2);                   % [T × 2]
    ref_cum = zeros(T, 1);
    for k = 2:T
        ref_cum(k) = ref_cum(k-1) + norm(ref_pos(k,:) - ref_pos(k-1,:));
    end
    ref_pct = 100 * ref_cum / max(path_length, eps);

    plot(t_ctrl, ref_pct, Color=color, LineWidth=line_width, LineStyle=':', ...
         DisplayName=strcat('$s$ ($N{=}',num2str(N),'$)'));
    plot(t_ctrl, actual_pct, Color=color, LineWidth=line_width, ...
         DisplayName=strcat('actual ($N{=}',num2str(N),'$)'));

    if N < 10  % draw the y-label only once
        ax.YAxis.Label.String  = '';
        ax.YAxis.Label.Visible = 'off';
        text(-1.3, 50, 'Path [\%]', ...
            'Rotation',90, 'VerticalAlignment','bottom', ...
            'HorizontalAlignment','center', ...
            'FontSize',1.0*font_size, 'Color','black', ...
            'FontWeight',font_weight, 'Interpreter','latex');
    end
    xlim([0, sim_length]);


    %% ---- Subplot 2: Compute time [s]  (log scale) ----------------------
    ax = subplot(fig_rows, fig_cols, 2);
    hold on;
    lgd = legend('Position',[0.56 0.71 0.1 0.05], Box='off', ...
                 BackgroundAlpha=0, FontSize=0.8*font_size, ...
                 FontWeight=font_weight, NumColumns=2, Orientation='horizontal');
    lgd.ItemTokenSize = [12, 10];
    set(gca, 'YScale','log');
    yticks([1e-4 1e-3 1e-2 1e-1 1]);
    yticklabels({'10^{-4}','10^{-3}','10^{-2}','10^{-1}','10^{0}'});

    if N > pathFG_max_N
        plot(t_ctrl, mpc_data, Color=color, LineWidth=line_width, ...
             DisplayName=strcat('Ungoverned MPC ($N{=}',num2str(N),'$)'));
    else
        plot(t_ctrl, pfg_data + mpc_data, Color=color, LineWidth=line_width, ...
             DisplayName=strcat('PathFG+MPC ($N{=}',num2str(N),'$)'));
        plot(t_ctrl, pfg_data, Color=color, LineWidth=line_width, LineStyle=':', ...
             DisplayName=strcat('PathFG ($N{=}',num2str(N),'$)'));
    end
    ylabel('Compute Time [s]', FontWeight=font_weight);
    xlim([0, sim_length]);


    %% ---- Subplot 3: Forward speed  v  [m/s] ----------------------------
    ax = subplot(fig_rows, fig_cols, 3);
    hold on;
    lgd = legend('Position',[0.67 0.40 0.1 0.05], Orientation='horizontal', ...
                 Box='off', BackgroundAlpha=0, FontWeight=font_weight);
    lgd.ItemTokenSize = [14, 10];
    plot(t_ctrl, u_data(:,2), Color=color, LineWidth=line_width, ...
         DisplayName=strcat('$N{=}',num2str(N),'$'));
    lgd.AutoUpdate = 'off';
    yline(u_max(2));
    yline(u_min(2));
    lgd.AutoUpdate = 'on';
    ylabel('Speed $v$ [m/s]', FontWeight=font_weight, Interpreter='latex');
    ytickformat('%.1f');
    xlim([0, sim_length]);
    ylim([u_min(2), u_max(2)]);


    %% ---- Subplot 4: Steering angle  θ  [rad] ---------------------------
    ax = subplot(fig_rows, fig_cols, 4);
    hold on;
    lgd = legend('Position',[0.67 0.18 0.1 0.05], Orientation='horizontal', ...
                 Box='off', BackgroundAlpha=0, FontWeight=font_weight);
    lgd.ItemTokenSize = [14, 10];
    ytickformat('%.1f');
    plot(t_ctrl, u_data(:,1), Color=color, LineWidth=line_width, ...
         DisplayName=strcat('$N{=}',num2str(N),'$'));
    lgd.AutoUpdate = 'off';
    yline(u_max(1));
    yline(u_min(1));
    lgd.AutoUpdate = 'on';
    ylabel('Steering $\theta$ [rad]', FontWeight=font_weight, Interpreter='latex');
    xlim([0, sim_length]);
    xlabel('Time [s]', FontWeight=font_weight);


    %% ---- Export ---------------------------------------------------------
    for fig_idx = 1:fig_count
        exportgraphics(figure(fig_idx), strcat('fig', num2str(fig_idx), '.pdf'), ...
            BackgroundColor='none', ContentType='vector');
    end

end