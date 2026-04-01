% Plot single prediction horizon
function plotGraphSingleN(reference_signal,...
    non_linear_full_states,...
    pathFG_time, ...
    MPC_time, ...
    control_inputs, ...
    colors,...
    goal, ...
    N, ...
    aux_ref_params, ...
    path, ...
    iterations, ...
    inner_controller_inclusion, ...
    mass, ...
    g, ...
    sim_length, ...
    u_min, ...
    u_max, ...
    x_min, ...
    x_max, ...
    solver)

    global fig1;

    fig_count = 1;
    color = colors(1); % primary color
    line_width = 3;
    downsample_rate = 100;
    buffer = 0.01; % distance (as a percentage) between the top/bottom of the subfigure to the topmost/bottomost plot

    %%

    figure(fig1);
    fig5_rows = 6;
    fig5_cols = 1;
    font = 'times'; % 'times' for Times New Roman and 'LM Roman 10' for Latin Modern Roman
    font_size = 18;
    font_weight = 'normal'; % either 'bold' or 'normal'
    set(groot, ... "groot" stands for "graphics root"
        ...
        defaultAxesFontSize   = 1 * font_size, ...
        defaultAxesFontName   = font, ...
        defaultAxesFontWeight = font_weight, ...
        ...
        defaultLegendInterpreter     = 'latex', ...
        defaultLegendLocation        = 'best', ...
        defaultLegendBox             = 'off', ...
        defaultLegendFontSize        = 1 * font_size, ... % the scaling not working
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

    ax = subplot(fig5_rows, fig5_cols, 1);
    hold on; legend;
    colororder({'#0072BD', '#ffA500'}); % NOTE hardcoded values
    non_linear_full_states_downsampled = downsample(non_linear_full_states.Data, downsample_rate);
    reference_signal_increased_size = zeros(size(reference_signal.Data, 1), 9);
    reference_signal_increased_size(:,1:4) = reference_signal.Data;
    error = non_linear_full_states_downsampled - reference_signal_increased_size;
    % non_linear_full_states_xyz_downsampled = downsample(non_linear_full_states.Data(:,1:3), downsample_rate);
    % reference_signal_xyz = reference_signal.Data(:,1:3);
    % error = non_linear_full_states_xyz_downsampled - reference_signal_xyz;
    error_norm = vecnorm(error, 2, 2); % take the norm of every row
    % error_norm = error;
    yyaxis left;
    ax.YAxis(1).Color = 'black';
    ytickformat('%.2f');
    legend('Position',[0.773 0.865 0.1 0.05], FontSize=font_size); 
    % plot(reference_signal.Time, error_norm, Color=colors(1), LineWidth=line_width, DisplayName=strcat('$\norm{x - \tilde x_s}$'));
   
    plot(reference_signal.Time, error_norm, ...
    'Color', colors(1), ...
    'LineWidth', line_width, ...
    'DisplayName', '$\|x - \tilde{x}_s\|$');  % Or '\lVert x - \tilde{x}_s \rVert'

    % Set LaTeX interpreter for the legend AFTER plotting
    legend('Interpreter', 'latex');


    ylabel('Error', Color='black');
    cur_ylim = ylim; % current y-axis limits
    ylim((1 + buffer) * cur_ylim);
    
    yyaxis right;
    ax.YAxis(2).Color = 'black';
    % convert s into a length
    if solver == 5
        % For solver 5, reference_signal already contains (x, y, z) positions
        ref_positions = reference_signal.Data(:, 1:3);
        length_from_start = getLengthFromStartXYZ(ref_positions);
        ref_path_length = length_from_start(end);
        path_percentage = 100 * length_from_start / ref_path_length;
    else
        length_from_start = zeros(1, numel(aux_ref_params.Data));
        for s_idx = 1:numel(aux_ref_params.Data)
            length_from_start(s_idx) = getLengthFromStart(aux_ref_params.Data(s_idx), path);
        end
        % total length of path
        path_length = 0;
        for node_idx = 1:size(path,1)-1
            path_length = path_length + norm(path(node_idx+1,:) - path(node_idx,:));
        end
        path_percentage = 100 * length_from_start / path_length; % normalize and convert to percentage
    end
    ytickformat('%.0f');
    axis = gca;
    ax.YAxis(2).FontSize = 1.0*font_size; % Y-axis tick labels only
    
    % Position the right Y lable at a customized position
    % 1. FIRST completely remove the default label
    ax.YAxis(2).Label.String = '';
    ax.YAxis(2).Label.Visible = 'off';
    % 2. THEN add a custom text label in exact position
    right_label_x = 20; % Adjust 0.08 as needed
    right_label_y = 50;
    text(right_label_x, right_label_y, ...
        'Path [\%]', ...
        'Rotation', 90, ...
        'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 1.0*font_size, ...
        'Color', 'black', ...
        'Interpreter', 'latex');

    if solver == 5
        plot(reference_signal.Time, path_percentage, Color=colors(2), LineWidth=line_width, DisplayName=strcat('$s$'));
    else
        plot(aux_ref_params.Time, path_percentage, Color=colors(2), LineWidth=line_width, DisplayName=strcat('$s$'));
    end
    xlim([0, sim_length]);
    ylim([0, (1 + buffer) * 100]); % make the graph slightly higher so that the line is visible

    subplot(fig5_rows, fig5_cols, 2);
    hold on;
    p_dot_max = sqrt(x_max(4)^2 + x_max(5)^2 + x_max(6)^2);
    p_dot = sqrt(non_linear_full_states.Data(:,4).^2 + non_linear_full_states.Data(:,5).^2 + non_linear_full_states.Data(:,6).^2);
    plot(non_linear_full_states.Time, p_dot, Color=color, LineWidth=line_width);
    % yline(p_dot_max);
    ylabel({'Speed [m/s]'});
    ytickformat('%.2f');
    xlim([0, sim_length]);

    subplot(fig5_rows, fig5_cols, 3);
    hold on; 
    lgd = legend('Position',[0.66 0.61 0.1 0.05], Orientation='horizontal', FontSize=1.05*font_size); % Location='northeast'
    lgd.AutoUpdate = 'off';
    % yline(x_max(7));
    % yline(x_min(7));
    lgd.AutoUpdate = 'on';
    ytickformat('%.2f');
    plot(non_linear_full_states.Time, non_linear_full_states.Data(:,7), Color=color, LineWidth=line_width, DisplayName='roll');
    plot(non_linear_full_states.Time, non_linear_full_states.Data(:,8), Color=colors(2), LineWidth=line_width, DisplayName='pitch');
    plot(non_linear_full_states.Time, non_linear_full_states.Data(:,9), Color=colors(3), LineWidth=line_width, DisplayName='yaw');
    ylabel({'Attitudes [rad]'});
    xlim([0, sim_length]);

    subplot(fig5_rows, fig5_cols, 4);
    hold on; 
    % lgd = legend('Position',[0.77 0.45 0.1 0.05], FontSize=1.0*font_size);
    % lgd.AutoUpdate = 'off';
    % % yline(u_max(1) + mass * g);
    % % yline(u_min(1) + mass * g);
    % lgd.AutoUpdate = 'on';
    ytickformat('%.2f');
    plot(control_inputs.Time, control_inputs.Data(1,:) + mass * g, Color=color, LineWidth=line_width);
    ylabel({'Total Thrust [N]'});
    xlim([0, sim_length]);

    subplot(fig5_rows, fig5_cols, 5);
    hold on; 
    lgd = legend('Position',[0.68 0.325 0.1 0.05], Orientation='horizontal', FontSize=1.3*font_size); 
    ytickformat('%.2f');
    plot(control_inputs.Time, control_inputs.Data(2,:), Color=color, LineWidth=line_width, DisplayName='$\omega_x$');
    plot(control_inputs.Time, control_inputs.Data(3,:), Color=colors(2), LineWidth=line_width, DisplayName='$\omega_y$');
    plot(control_inputs.Time, control_inputs.Data(4,:), Color=colors(3), LineWidth=line_width, DisplayName='$\omega_z$');
    ylabel({'Angular'; 'Speed [rad/s]'});
    xlim([0, sim_length]);

    subplot(fig5_rows, fig5_cols, 6);
    hold on; 
    % yscale log;
    % set(gca, 'YScale', 'log');  % Convert to log
    % yticks([1e-3 1e-2]);
    % yticklabels({'10^{-3}','10^{-2}'});
    legend('Position',[0.38 0.116 0.1 0.05], FontSize=1*font_size); 
    plot(pathFG_time.Time, pathFG_time.Data + MPC_time.Data, Color=colors(2), LineWidth=line_width,DisplayName=strcat('PathFG+MPC ($N{=}',num2str(N),'$)'));
        plot(pathFG_time.Time, pathFG_time.Data, Color=color, LineWidth=line_width, DisplayName=strcat('PathFG ($N{=}',num2str(N),'$)'));
    ytickformat('%.3f');
    ylabel({'Compute Time [s]'});
    xlabel({'', 'Time [s]'})
    xlim([0, sim_length]);


    %%

    % exportation
    for fig_idx = 1:fig_count
        exportgraphics(figure(fig_idx), strcat('fig', num2str(fig_idx), '.pdf'), BackgroundColor='none', ContentType='vector');
        % saveas(figure(fig_idx), strcat('fig', num2str(fig_idx), '.svg'))
        % saving as pdf doesn't support font embedding for non-standard fonts
    end
    % export_fig 'fig5.pdf'

end