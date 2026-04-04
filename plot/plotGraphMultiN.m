% Plot multiple prediction horizons
function plotGraphMultiN( ...
    reference_signal,...
    non_linear_full_states,...
    PathFG_time, ...
    MPC_time, ...
    control_inputs, ...
    color,...
    goal, ...
    u_min, ...
    u_max, ...
    x_max,...
    N, ...
    iterations, ...
    N_idx, ...
    aux_ref_params, ...
    path, ...
    MPC_x0, ...
    mass, ...
    g, ...
    PathFG_max_N, ...
    sim_length,...
    solver)

    global fig1;

    fig_count = 1;
    line_width = 2;   

    %%
    
    figure(fig1);
    fig5_rows = 5;
    fig5_cols = 1;
    font = 'times';
    font_size = 12;
    font_weight = 'normal';
    set(groot, ... "groot" stands for "graphics root"
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


    % subfigure
    ax = subplot(fig5_rows, fig5_cols, 1);
    hold on;
    % lgd = legend('Position',[0.74 0.837 0.1 0.05], Box = 'off', BackgroundAlpha = 0, FontWeight=font_weight, FontSize=0.82*font_size);
    lgd = legend('Position',[0.65 0.837 0.1 0.05], NumColumns=2, Orientation='horizontal',Box = 'off', BackgroundAlpha = 0, FontWeight=font_weight, FontSize=0.82*font_size);   
    lgd.ItemTokenSize = [12, 10]; 
    % lgd.Orientation = 'horizontal';
   
    if solver == 5
        % For solver 5, reference_signal already contains (x, y, z) positions
        ref_positions = reference_signal.Data(:, 1:3); % N x 3 matrix
        length_from_start = getLengthFromStartXYZ(ref_positions);
        ref_path_length = length_from_start(end); % total arc length of reference
        path_percentage = 100 * length_from_start / ref_path_length;
    else
        % convert s into a length
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
    % get the path percentage of the actual state
    actual_path = MPC_x0.Data(1:3,:)'; % get the path p (the position) has taken
    actual_length_from_start = getLengthFromStartXYZ(actual_path);
    actual_path_length = actual_length_from_start(end); % length of the path taken by p
    actual_path_percentage = 100 * actual_length_from_start / actual_path_length; % convert to percentage


    
    if solver == 5
        plot(reference_signal.Time, path_percentage, Color=color, LineWidth=line_width, LineStyle=":", DisplayName=strcat('$s$ ($N^*{=}',num2str(N),'$)'));
        plot(MPC_x0.Time, actual_path_percentage, Color=color, LineWidth=line_width, DisplayName=strcat('actual ($N^*{=}',num2str(N),'$)'));
    elseif N > PathFG_max_N
        plot(aux_ref_params.Time, path_percentage, Color=color, LineWidth=line_width, LineStyle=":", DisplayName=strcat('$s$ ($N^*{=}',num2str(N),'$)'));
        plot(MPC_x0.Time, actual_path_percentage, Color=color, LineWidth=line_width, DisplayName=strcat('actual ($N^*{=}',num2str(N),'$)'));
    else
        plot(aux_ref_params.Time, path_percentage, Color=color, LineWidth=line_width, LineStyle=":", DisplayName=strcat('$s$ ($N{=}',num2str(N),'$)'));
        plot(MPC_x0.Time, actual_path_percentage, Color=color, LineWidth=line_width, DisplayName=strcat('actual ($N{=}',num2str(N),'$)'));
    end


    
    % Position the right Y lable at a customized position
    if N < 10  % only draw once
        % 1. FIRST completely remove the default label
        ax.YAxis.Label.String = '';
        ax.YAxis.Label.Visible = 'off';
        % 2. THEN add a custom text label in exact position
        right_label_x = -1.3; % Adjust 0.08 as needed
        right_label_y = 50;
        text(right_label_x, right_label_y, ...
            'Path [\%]', ...
            'Rotation', 90, ...
            'VerticalAlignment', 'bottom', ...
            'HorizontalAlignment', 'center', ...
            'FontSize', 1.0*font_size, ...
            'Color', 'black', ...
            'FontWeight',font_weight,...
            'Interpreter', 'latex');
    end

    xlim([0, sim_length]);



    % subfigure
    ax = subplot(fig5_rows, fig5_cols, 2);
    hold on;
    lgd = legend('Position', [0.52 0.72 0.1 0.05], Box = 'off', BackgroundAlpha = 0, FontSize=0.8*font_size, FontWeight = font_weight, NumColumns=2, Orientation='horizontal');
    lgd.ItemTokenSize = [12, 10]; 
    yscale log;
    set(gca, 'YScale', 'log');  % Convert to log
    yticks([1e-3 1e-2 1e-1 1e0]);
    yticklabels({'10^{-3}','10^{-2}','10^{-1}','10^{0}'});
    % set(gca, 'TickLabelInterpreter', 'latex');
    if solver == 5
        plot(PathFG_time.Time, MPC_time.Data, Color=color, LineWidth=line_width, DisplayName=strcat('MPCa ($N^*{=}',num2str(N),'$)'));
    elseif N > PathFG_max_N
        plot(PathFG_time.Time, MPC_time.Data, Color=color, LineWidth=line_width, DisplayName=strcat('Ungoverned MPC ($N^*{=}',num2str(N),'$)'));
    else
        plot(PathFG_time.Time, PathFG_time.Data + MPC_time.Data, Color=color, LineWidth=line_width, DisplayName=strcat('PathFG+MPC ($N{=}',num2str(N),'$)'));
        plot(PathFG_time.Time, PathFG_time.Data, Color=color, LineWidth=line_width, LineStyle=':', DisplayName=strcat('PathFG ($N{=}',num2str(N),'$)'));
    end
    % ylabel({'Compute', 'Time [s]'}, FontWeight=font_weight);
    ylabel('Compute Time [s]', FontWeight=font_weight);
    xlim([0, sim_length]);
    ylim([0 1e2]); 



    % subfigure
    ax = subplot(fig5_rows, fig5_cols, 3);
    hold on;
    lgd = legend('Position', [0.59 0.525 0.1 0.05], Orientation='horizontal', Box = 'off', BackgroundAlpha = 0, FontWeight = font_weight); 
    lgd.ItemTokenSize = [14, 10]; 
    p_dot = sqrt(non_linear_full_states.Data(:, 4).^2 + non_linear_full_states.Data(:, 5).^2 + non_linear_full_states.Data(:, 6).^2);
    if solver == 5 || N > PathFG_max_N
        plot(non_linear_full_states.Time, p_dot, Color=color, LineWidth=line_width, DisplayName=strcat('$N^*{=}',num2str(N),'$'));
    else
        plot(non_linear_full_states.Time, p_dot, Color=color, LineWidth=line_width, DisplayName=strcat('$N{=}',num2str(N),'$'));
    end
    p_dot_max = sqrt(x_max(4)^2 + x_max(5)^2 + x_max(6)^2);
    lgd.AutoUpdate = 'off';
    yline(p_dot_max);
    lgd.AutoUpdate = 'on';
    ylabel('Speed [m/s]', FontWeight=font_weight);
    ytickformat('%.1f');
    xlim([0, sim_length]);
    ylim([0, p_dot_max]);

    % subfigure
    ax = subplot(fig5_rows, fig5_cols, 4);
    hold on;
    lgd = legend('Position', [0.59 0.35 0.1 0.05], Orientation='horizontal', Box = 'off', BackgroundAlpha = 0, FontWeight = font_weight);
    lgd.ItemTokenSize = [14, 10]; 
    ytickformat('%.1f');
    if solver == 5 || N > PathFG_max_N
        plot(control_inputs.Time, control_inputs.Data(1,:) + mass * g, Color=color, LineWidth=line_width, DisplayName=strcat('$N^*{=}',num2str(N),'$'));
    else
        plot(control_inputs.Time, control_inputs.Data(1,:) + mass * g, Color=color, LineWidth=line_width, DisplayName=strcat('$N{=}',num2str(N),'$'));
    end
    lgd.AutoUpdate = 'off';
    yline(u_max(1) + mass * g);
    yline(u_min(1) + mass * g);
    lgd.AutoUpdate = 'on';
    ylabel({'Total Thrust [N]'}, FontWeight=font_weight);
    xlim([0, sim_length]);


    % subfigure
    ax = subplot(fig5_rows, fig5_cols, 5);
    hold on;
    lgd = legend('Position', [0.59 0.18 0.1 0.05], Orientation='horizontal', Box = 'off', BackgroundAlpha = 0, FontWeight = font_weight);
    lgd.ItemTokenSize = [14, 10]; 
    angular_speeds = sqrt(control_inputs.Data(2,:).^2 + control_inputs.Data(3,:).^2 + control_inputs.Data(4,:).^2);
    ytickformat('%.1f');
    if solver == 5 || N > PathFG_max_N
        plot(control_inputs.Time, angular_speeds, Color=color, LineWidth=line_width, DisplayName=strcat('$N^*{=}',num2str(N),'$'));
    else
        plot(control_inputs.Time, angular_speeds, Color=color, LineWidth=line_width, DisplayName=strcat('$N{=}',num2str(N),'$'));
    end
    
    lgd.AutoUpdate = 'off';
    u_upper = sqrt(u_max(2)^2 + u_max(3)^2 + u_max(4)^2);
    yline(u_upper);
    lgd.AutoUpdate = 'on';
    ylabel({'Angular'; 'Speed [rad/s]'}, FontWeight=font_weight);
    xlim([0, sim_length]);
    ylim([0, u_upper]);
    xlabel('Time [s]', FontWeight=font_weight);
    


    %%

    % exportation
    for fig_idx = 1:fig_count
        exportgraphics(figure(fig_idx), strcat('fig', num2str(fig_idx), '.pdf'),  BackgroundColor='none', ContentType='vector');
    end

     

end