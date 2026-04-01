% Plot multiple prediction horizons
function plotGraphMultiPlanner(reference_signal,...
    non_linear_full_states,...
    pathFG_time, ...
    MPC_time, ...
    control_inputs, ...
    color,...
    goal, ...
    u_min, ...
    u_max, ...
    N, ...
    iterations, ...
    aux_ref_params, ...
    MPC_x0,...
    path, ...
    path_planner, ...
    mass, ...
    g, ...
    solver)

    global fig1;

    line_width = 3;

    %% ===== Figure setup =====
    figure(fig1);

    % Make figure physically larger (CRITICAL for font size)
    set(gcf, 'Units', 'inches', 'Position', [1 1 8 4]);
    set(gcf, 'PaperPositionMode', 'auto');

    fig_rows = 2;
    fig_cols = 1;

    %% ===== Global style =====
    font = 'times';
    font_size = 14;  
    font_weight = 'normal';

    set(groot, ...
        'defaultAxesFontSize',   font_size, ...
        'defaultAxesFontName',   font, ...
        'defaultAxesFontWeight', font_weight, ...
        ...
        'defaultLegendInterpreter',     'latex', ...
        'defaultLegendLocation',        'best', ...
        'defaultLegendBox',             'off', ...
        'defaultLegendFontSize',        font_size, ...
        'defaultLegendFontWeight',      font_weight, ...
        'defaultLegendBackgroundAlpha', 0, ...
        ...
        'defaultConstantLineColor',     'black', ...
        'defaultConstantLineLineStyle', ':', ...
        'defaultConstantLineLineWidth', line_width, ...
        ...
        'defaultTextInterpreter', 'latex', ...
        'defaultTextFontWeight',  font_weight ...
    );

    %% ===== Subplot 1: Path percentage =====
    ax = subplot(fig_rows, fig_cols, 1);
    hold on;
    ax.FontSize = font_size;

    lgd = legend('Position',[0.7 0.76 0.15 0.08], ...
        'Box','off', ...
        'BackgroundAlpha',0, ...
        'FontWeight',font_weight, ...
        'FontSize',0.95*font_size);
        lgd.ItemTokenSize = [18, 10];

    % ---- Compute reference path percentage ----

    % get the path percentage of the actual state
    actual_path = MPC_x0.Data(1:3,:)'; % get the path p (the position) has taken
    actual_length_from_start = getLengthFromStartXYZ(actual_path);
    actual_path_length = actual_length_from_start(end); % length of the path taken by p
    actual_path_percentage = 100 * actual_length_from_start / actual_path_length; % convert to percentage
  
    % length_from_start = zeros(1, numel(aux_ref_params.Data));
    % for s_idx = 1:numel(aux_ref_params.Data)
    %     length_from_start(s_idx) = getLengthFromStart(aux_ref_params.Data(s_idx), path);
    % end
    % 
    % path_length = 0;
    % for node_idx = 1:size(path,1)-1
    %     path_length = path_length + norm(path(node_idx+1,:) - path(node_idx,:));
    % end
    % path_percentage = 100 * length_from_start / path_length;
    
    % Set x-axis limits
    xlim([0 pathFG_time.Time(end)]);

    % ---- Plot ----
    plot(MPC_x0.Time, actual_path_percentage, ...
        'Color', color, ...
        'LineWidth', line_width, ...
        'DisplayName', strcat('actual (', path_planner, ')'));
        % 'DisplayName', strcat('$s$ (', path_planner, ', $N{=}', num2str(N), '$)'));
        

    ylabel('Path [\%]');
    xlabel('Time [s]');

    %% ===== Subplot 2: Computation time =====
    ax = subplot(fig_rows, fig_cols, 2);
    hold on;
    ax.FontSize = font_size;

    lgd = legend('Position',[0.68 0.31 0.15 0.08], ...
        'Box','off', ...
        'BackgroundAlpha',0, ...
        'FontWeight',font_weight, ...
        'FontSize',0.9*font_size);
    lgd.ItemTokenSize = [18, 10];
    % Set x-axis limits
    xlim([0 pathFG_time.Time(end)]);
    ytickformat('%.2f');

    % ---- Plot ----
    plot(pathFG_time.Time, pathFG_time.Data + MPC_time.Data, ...
        'Color', color, ...
        'LineWidth', line_width, ...
        'DisplayName', strcat('PathFG+MPC (', path_planner, ')'));

    plot(pathFG_time.Time, pathFG_time.Data, ...
        'Color', color, ...
        'LineWidth', line_width, ...
        'LineStyle', ':', ...
        'DisplayName', strcat('PathFG (', path_planner, ')'));

    ylabel('Compute time [s]');
    xlabel('Time [s]');


    %% ===== Export as PNG =====
    % print(fig1, 'fig1_IEEE', '-dpng', '-r600'); 
    exportgraphics(fig1, 'fig1_IEEE.png', 'Resolution', 600);

    %%

    % figure(fig1);
    % fig1_rows = 2;
    % fig1_cols = 1;
    % 
    % % ax = subplot(fig1_rows, fig1_cols, 1);
    % % ax.FontSize = 20;
    % % hold on;
    % % lgd = legend;
    % % lgd.Location = 'northwest'; lgd.Orientation = 'horizontal';
    % % lgd.FontSize = 25; lgd.FontWeight = 'bold';
    % % if N_idx == 1
    % %     yline(goal(1), ':black', LineWidth=line_width, DisplayName='r');
    % % end
    % % plot(reference_signal.Time, reference_signal.Data(:,1), Color=color, LineWidth=line_width, LineStyle=":", DisplayName=strcat('v (N=',num2str(N),')'));
    % % plot(non_linear_full_states.Time, non_linear_full_states.Data(:,1), Color=color, LineWidth=line_width, DisplayName=strcat('actual (N=',num2str(N),')'));
    % % ylabel('x [m]', FontWeight='bold');
    % % xlabel('Time [s]', FontWeight='bold');
    % % 
    % % ax = subplot(fig1_rows, fig1_cols, 2);
    % % hold on;
    % % ax.FontSize = 20;
    % % yline(goal(2), ':black', LineWidth=line_width);
    % % plot(reference_signal.Time, reference_signal.Data(:,2), Color=color, LineWidth=line_width, LineStyle=":");
    % % plot(non_linear_full_states.Time, non_linear_full_states.Data(:,2), Color=color, LineWidth=line_width);
    % % ylabel('y [m]', FontWeight='bold');
    % % xlabel('Time [s]', FontWeight='bold');
    % % 
    % % ax = subplot(fig1_rows, fig1_cols, 3);
    % % hold on;
    % % ax.FontSize = 20;
    % % yline(goal(3), ':black', LineWidth=line_width);
    % % plot(reference_signal.Time, reference_signal.Data(:,3), Color=color, LineWidth=line_width, LineStyle=":");
    % % plot(non_linear_full_states.Time, non_linear_full_states.Data(:, 3), Color=color, LineWidth=line_width);
    % % ylabel('z [m]', FontWeight='bold');
    % % xlabel('Time [s]', FontWeight='bold');
    % 
    % ax = subplot(fig1_rows, fig1_cols, 1);
    % hold on;
    % ax.FontSize = 20;
    % lgd = legend;
    % lgd.Location = 'northwest'; lgd.Orientation = 'horizontal';
    % lgd.FontSize = 25; lgd.FontWeight = 'bold';
    % % convert s into a length
    % length_from_start = zeros(1, numel(aux_ref_params.Data));
    % for s_idx = 1:numel(aux_ref_params.Data)
    %     length_from_start(s_idx) = getLengthFromStart(aux_ref_params.Data(s_idx), path);
    % end
    % % total length of path
    % path_length = 0;
    % for node_idx = 1:size(path,1)-1
    %     path_length = path_length + norm(path(node_idx+1,:) - path(node_idx,:));
    % end
    % s = 100 * length_from_start / path_length; % normalize and convert to percentage
    % plot(aux_ref_params.Time, s, Color=color, LineWidth=line_width, DisplayName=path_planner);
    % ylabel('path percentage [%]', FontWeight='bold');
    % xlabel('Time [s]', FontWeight='bold');
    % 
    % ax = subplot(fig1_rows, fig1_cols, 2);
    % hold on;
    % ax.FontSize = 20;
    % % yline(0, ':black', LineWidth=line_width);
    % % p_dot = sqrt(non_linear_full_states.Data(:, 4).^2 + non_linear_full_states.Data(:, 5).^2 + non_linear_full_states.Data(:, 6).^2);
    % % plot(non_linear_full_states.Time, p_dot, Color=color, LineWidth=line_width);
    % % ylabel('Speed [m/s]', FontWeight='bold');
    % % xlabel('Time [s]', FontWeight='bold');
    % lgd = legend;
    % lgd.Location = 'northwest'; lgd.Orientation = 'horizontal';
    % lgd.FontSize = 25; lgd.FontWeight = 'bold';
    % plot(pathFG_time.Time, pathFG_time.Data + MPC_time.Data, Color=color, LineWidth=line_width, DisplayName=strcat('pathFG+MPC (',path_planner,')'));
    % plot(pathFG_time.Time, pathFG_time.Data, Color=color, LineWidth=line_width, LineStyle=':', DisplayName=strcat('pathFG (',path_planner,')'));
    % ylabel('Computation Time [s]', FontWeight='bold');
    % xlabel('Time [s]', FontWeight='bold');
    % 
    % print('fig1', '-depsc');


    %%
    % 
    % figure(fig2);
    % fig2_rows = 2;
    % fig2_cols = 1;
    % 
    % ax = subplot(fig2_rows, fig2_cols, 1);
    % hold on;
    % ax.FontSize = 20;
    % % if N_idx == 1
    % %     yline(u_min(1), ':black', LineWidth=line_width, DisplayName='Constraints');
    % % end
    % plot(control_inputs.Time, control_inputs.Data(1, :)+mass*g, Color=color, LineWidth=line_width, DisplayName=path_planner);
    % lgd = legend;
    % lgd.Location = 'northwest'; lgd.Orientation = 'horizontal';
    % lgd.FontSize = 25; lgd.AutoUpdate = 'off'; lgd.FontWeight = 'bold';
    % % yline(u_max(1), ':black', LineWidth=line_width);
    % lgd.AutoUpdate = 'on';
    % ylabel('total thrust [N]', FontWeight='bold');
    % xlabel('Time [s]', FontWeight='bold');
    % 
    % ax = subplot(fig2_rows, fig2_cols, 2);
    % hold on;
    % ax.FontSize = 20;
    % % lgd = legend;
    % % lgd.Location = 'northwest'; lgd.Orientation = 'horizontal';
    % % lgd.FontSize = 25; lgd.FontWeight = 'bold';
    % angular_speeds = sqrt(control_inputs.Data(2,:).^2 + control_inputs.Data(3,:).^2 + control_inputs.Data(4,:).^2);
    % plot(control_inputs.Time, angular_speeds, Color=color, LineWidth=line_width);
    % ylabel('angular speed [rad/s]', FontWeight='bold');
    % xlabel('Time [s]', FontWeight='bold');
    % 
    % % ax = subplot(fig2_rows, fig2_cols, 2);
    % % hold on;
    % % ax.FontSize = 20;
    % % % yline(u_max(2), ':black', LineWidth=line_width);
    % % % yline(u_min(2), ':black', LineWidth=line_width);
    % % plot(control_inputs.Time, control_inputs.Data(2, :), Color=color, LineWidth=line_width);
    % % ylabel('Wx [rad/s]', FontWeight='bold');
    % % xlabel('Time [s]', FontWeight='bold');
    % % 
    % % ax = subplot(fig2_rows, fig2_cols, 3);
    % % hold on;
    % % ax.FontSize = 20;
    % % % yline(u_max(3), ':black', LineWidth=line_width);
    % % % yline(u_min(3), ':black', LineWidth=line_width);
    % % plot(control_inputs.Time, control_inputs.Data(3, :), Color=color, LineWidth=line_width);
    % % ylabel('Wy [rad/s]', FontWeight='bold');
    % % xlabel('Time [s]', FontWeight='bold');
    % % 
    % % ax = subplot(fig2_rows, fig2_cols, 4);
    % % hold on;
    % % ax.FontSize = 20;
    % % % yline(u_max(4), ':black', LineWidth=line_width);
    % % % yline(u_min(4), ':black', LineWidth=line_width);
    % % plot(control_inputs.Time, control_inputs.Data(4, :), Color=color, LineWidth=line_width);
    % % ylabel('Wz [rad/s]', FontWeight='bold');
    % % xlabel('Time [s]', FontWeight='bold');
    % 
    % print('fig2', '-depsc');
    % 
    % 
    % %%
    % 
    % figure(fig3);
    % fig3_rows = 2;
    % fig3_cols = 1;
    % 
    % ax = subplot(fig3_rows, fig3_cols, 1);
    % hold on;
    % ax.FontSize = 20;
    % lgd = legend;
    % lgd.Location = 'northwest'; lgd.Orientation = 'horizontal';
    % lgd.FontSize = 25; lgd.FontWeight = 'bold';
    % plot(pathFG_time.Time, MPC_time.Data, Color=color, LineWidth=line_width, DisplayName=strcat('MPC (',path_planner,')'));
    % plot(pathFG_time.Time, pathFG_time.Data + MPC_time.Data, Color=color, LineWidth=line_width, LineStyle=':', DisplayName=strcat('pathFG+MPC (',path_planner,')'));
    % ylabel('Computation Time [s]', FontWeight='bold');
    % xlabel('Time [s]', FontWeight='bold');
    % 
    % ax = subplot(fig3_rows, fig3_cols, 2);
    % hold on;
    % ax.FontSize = 20;
    % lgd = legend;
    % lgd.Location = 'northwest'; lgd.Orientation = 'horizontal';
    % lgd.FontSize = 25; lgd.FontWeight = 'bold';
    % plot(iterations.Time, iterations.Data, Color=color, LineWidth=line_width, DisplayName=path_planner);
    % ylabel('Optimization Iterations', FontWeight='bold');
    % xlabel('Time [s]', FontWeight='bold');
    % 
    % print('fig3', '-depsc');


end