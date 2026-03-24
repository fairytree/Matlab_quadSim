% Plot single prediction horizon
function plotGraphMultiGovernor(reference_signal,...
    non_linear_full_states,...
    governor_time_data, ...
    MPC_time, ...
    control_inputs, ...
    colors,...
    goal, ...
    N, ...
    aux_ref_params, ...
    path, ...
    mass, ...
    g, ...
    sim_length, ...
    u_min, ...
    u_max, ...
    x_min, ...
    x_max, ...
    ref_source)

    if(ref_source == 1)
        return;
    end

    global fig1;

    fig_count = 1;
    color = colors(1); % primary color
    line_width = 3;
    downsample_rate = 100;
    buffer = 0.01; % distance (as a percentage) between the top/bottom of the subfigure to the topmost/bottomost plot

    %%th

    figure(fig1);
    fig5_rows = 6;
    fig5_cols = 1;
    font = 'times'; % 'times' for Times New Roman and 'LM Roman 10' for Latin Modern Roman
    font_size = 12;
    font_weight = 'normal'; % either 'bold' or 'normal'
    set(groot, ... "groot" stands for "graphics root"
        ...
        defaultAxesFontSize   = font_size, ...
        defaultAxesFontName   = font, ...
        defaultAxesFontWeight = font_weight, ...
        ...
        defaultLegendInterpreter     = 'latex', ...
        defaultLegendLocation        = 'best', ...
        defaultLegendBox             = 'off', ...
        defaultLegendFontSize        = 1.0 * font_size, ... % the scaling not working
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

    % Line style based on reference source
    switch ref_source
        case 1
            line_style = '--';   % dashed
            line_width = 1.5;
            show_legend = false;
        case 2
            line_style = ':';    % dotted
            line_width = 1.5;
            show_legend = true;
        case 3
            line_style = '-';    % solid
            line_width = 2;
            show_legend = true;
        otherwise
            line_style = '-.';   % dash-dot (fallback)
    end


    ax = subplot(fig5_rows, fig5_cols, 1);
    hold on; legend;
    colororder({'#0072BD', '#ffA500'}); % NOTE hardcoded values
    non_linear_full_states_downsampled = downsample(non_linear_full_states.Data, downsample_rate);
    reference_signal_increased_size = zeros(size(reference_signal.Data, 1), 9);
    reference_signal_increased_size(:,1:4) = reference_signal.Data;
    % error = non_linear_full_states_downsampled - reference_signal_increased_size;
    disp("size of non_linear_full_states_downsampled");
    disp(size(non_linear_full_states_downsampled));
    disp("size of non_linear_full_states.Data");
    disp(size(non_linear_full_states.Data));
    disp("size of reference_signal_increased_size");
    disp(size(reference_signal_increased_size));
    if(ref_source == 1)
        error = non_linear_full_states.Data(:, 1:9) - reference_signal_increased_size; % Note!!! ERG is real-time
    else
        error = non_linear_full_states_downsampled(:, 1:9) - reference_signal_increased_size;
    end
    % non_linear_full_states_xyz_downsampled = downsample(non_linear_full_states.Data(:,1:3), downsample_rate);
    % reference_signal_xyz = reference_signal.Data(:,1:3);
    % error = non_linear_full_states_xyz_downsampled - reference_signal_xyz;
    error_norm = vecnorm(error, 2, 2); % take the norm of every row
    % error_norm = error;
    yyaxis left;
    ax.YAxis(1).Color = 'black';
    ytickformat('%.2f');
    lgd = legend('Position',[0.728 0.85 0.1 0.05], FontSize=font_size); 
    lgd.ItemTokenSize = [9, 10];
    % plot(reference_signal.Time, error_norm, Color=colors(1), LineWidth=line_width, DisplayName=strcat('$\norm{x - \tilde x_s}$'));
   
    h = plot(reference_signal.Time, error_norm, ...
    'Color', colors(1), ...
    'LineStyle', line_style, ...
    'LineWidth', line_width);  % Or '\lVert x - \tilde{x}_s \rVert'

    if show_legend && ref_source == 2
        h.DisplayName = '$\|x - \tilde{x}_s\|_{ERG}$'; % Or '\lVert x - \tilde{x}_s \rVert'
    elseif show_legend && ref_source == 3
        h.DisplayName = '$\|x - \tilde{x}_s\|_{PathFG}$'; % Or '\lVert x - \tilde{x}_s \rVert'
    else
        h.HandleVisibility = 'off';
    end


    % Set LaTeX interpreter for the legend AFTER plotting
    legend('Interpreter', 'latex');


    ylabel('Error', Color='black');
    cur_ylim = ylim; % current y-axis limits
    ylim((1 + 7 * buffer) * cur_ylim);
    
    yyaxis right;
    ax.YAxis(2).Color = 'black';
    % convert s into a length
    length_from_start = zeros(1, numel(aux_ref_params.Data));
    idx = 1;
    for s_idx = 1:numel(aux_ref_params.Data)
        length_from_start(s_idx) = getLengthFromStart(aux_ref_params.Data(s_idx), path);
    end
    % total length of path
    path_length = 0;
    for node_idx = 1:size(path,1)-1
        path_length = path_length + norm(path(node_idx+1,:) - path(node_idx,:));
    end
    % s = length_from_start / path_length; % normalize
    path_percentage = 100 * length_from_start / path_length; % normalize and convert to percentage
    ytickformat('%.0f');
    axis = gca;
    ax.YAxis(2).FontSize = 1.0*font_size; % Y-axis tick labels only
    
    % Position the right Y lable at a customized position
    % 1. FIRST completely remove the default label
    ax.YAxis(2).Label.String = '';
    ax.YAxis(2).Label.Visible = 'off';
    % 2. THEN add a custom text label in exact position
    right_label_x = 33.7; % Adjust 0.08 as needed
    right_label_y = 50;
    text(right_label_x, right_label_y, ...
        'Path [\%]', ...
        'Rotation', 90, ...
        'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 1.0 * font_size, ...
        'Color', 'black', ...
        'Interpreter', 'latex');

    h = plot(aux_ref_params.Time, path_percentage, Color=colors(2), LineStyle=line_style, LineWidth=line_width);
    if show_legend && ref_source == 2
        h.DisplayName = '$s_{ERG}$'; 
    elseif show_legend && ref_source == 3
        h.DisplayName = '$s_{PathFG}$';
    else
        h.HandleVisibility = 'off';
    end
    
    xlim([0, sim_length]);
    ylim([0, (1 + buffer) * 100]); % make the graph slightly higher so that the line is visible

    subplot(fig5_rows, fig5_cols, 2);
    hold on;
    p_dot_max = sqrt(x_max(4)^2 + x_max(5)^2 + x_max(6)^2);
    p_dot = sqrt(non_linear_full_states.Data(:,4).^2 + non_linear_full_states.Data(:,5).^2 + non_linear_full_states.Data(:,6).^2);
    h = plot(non_linear_full_states.Time, p_dot, Color=color,LineStyle=line_style, LineWidth=line_width);
    % yline(p_dot_max);
    ylabel({'Speed [m/s]'});
    ytickformat('%.2f');
    xlim([0, sim_length]);
    lgd = legend('Position',[0.728 0.73 0.1 0.05], FontSize=font_size); 
    lgd.ItemTokenSize = [12, 10];
    if show_legend && ref_source == 2
        h.DisplayName = '$speed_{ERG}$';
    elseif show_legend && ref_source == 3
        h.DisplayName = '$speed_{PathFG}$';
    else
        h.HandleVisibility = 'off';
    end

    subplot(fig5_rows, fig5_cols, 3);
    hold on; 
    lgd = legend('Position',[0.55 0.6 0.1 0.05], Orientation='horizontal', NumColumns=3, FontSize=font_size); % Location='northeast'
    lgd.ItemTokenSize = [12, 10];
    lgd.AutoUpdate = 'off';
    % yline(x_max(7));
    % yline(x_min(7));
    lgd.AutoUpdate = 'on';
    ytickformat('%.2f');
    h = plot(non_linear_full_states.Time, non_linear_full_states.Data(:,7), Color=color, LineStyle=line_style, LineWidth=line_width);
    if show_legend && ref_source == 2
        h.DisplayName = '$roll_{ERG}$';
    elseif show_legend && ref_source == 3
        h.DisplayName = '$roll_{PathFG}$';  
    else
        h.HandleVisibility = 'off';
    end
    
    h = plot(non_linear_full_states.Time, non_linear_full_states.Data(:,8), Color=colors(2), LineStyle=line_style, LineWidth=line_width);
    if show_legend && ref_source == 2
        h.DisplayName = '$pitch_{ERG}$';
    elseif show_legend && ref_source == 3
        h.DisplayName = '$pitch_{PathFG}$';  
    else
        h.HandleVisibility = 'off';
    end
    
    h = plot(non_linear_full_states.Time, non_linear_full_states.Data(:,9), Color=colors(3), LineStyle=line_style, LineWidth=line_width);
    if show_legend && ref_source == 2
        h.DisplayName = '$yaw_{ERG}$';
    elseif show_legend && ref_source == 3
        h.DisplayName = '$yaw_{PathFG}$';  
    else
        h.HandleVisibility = 'off';
    end
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

    disp("control_input.Time");
    disp(size(control_inputs.Time));
    disp("control_inputs.Data");
    disp(size(control_inputs.Data));  
    if(ref_source == 2)
        h = plot(control_inputs.Time, control_inputs.Data(:,1) + mass * g, Color=color, LineStyle=line_style, LineWidth=line_width);
    else
        h = plot(control_inputs.Time, control_inputs.Data(1,:) + mass * g, Color=color, LineStyle=line_style, LineWidth=line_width);
    end
    lgd = legend('Position',[0.62 0.45 0.1 0.05], Orientation='horizontal', NumColumns=3, FontSize=font_size); % Location='northeast'
    lgd.ItemTokenSize = [12, 10];
    if show_legend && ref_source == 2
        h.DisplayName = '$thrust_{ERG}$';
    elseif show_legend && ref_source == 3
        h.DisplayName = '$thrust_{PathFG}$';
    else
        h.HandleVisibility = 'off';
    end

    ylabel({'Total'; 'Thrust [N]'});
    xlim([0, sim_length]);

    subplot(fig5_rows, fig5_cols, 5);
    hold on; 
    lgd = legend('Position',[0.57 0.315 0.1 0.05], Orientation='horizontal', NumColumns=3, FontSize=font_size); 
    lgd.ItemTokenSize = [12, 10];
    ytickformat('%.2f');
     if(ref_source == 2)
        h1 = plot(control_inputs.Time, control_inputs.Data(:, 2), Color=color, LineStyle=line_style, LineWidth=line_width);
        h2 = plot(control_inputs.Time, control_inputs.Data(:, 3), Color=colors(2), LineStyle=line_style, LineWidth=line_width);
        h3 = plot(control_inputs.Time, control_inputs.Data(:, 4), Color=colors(3), LineStyle=line_style, LineWidth=line_width);

        if show_legend
           h1.DisplayName = '${\omega_x}_{ERG}$';    
           h2.DisplayName = '${\omega_y}_{ERG}$';
           h3.DisplayName = '${\omega_z}_{ERG}$';
        else 
           h1.HandleVisibility = 'off';
           h2.HandleVisibility = 'off';
           h3.HandleVisibility = 'off';
        end
     else
        h1 = plot(control_inputs.Time, control_inputs.Data(2, :), Color=color, LineStyle=line_style, LineWidth=line_width);
        h2 = plot(control_inputs.Time, control_inputs.Data(3, :), Color=colors(2), LineStyle=line_style, LineWidth=line_width);
        h3 = plot(control_inputs.Time, control_inputs.Data(4, :), Color=colors(3), LineStyle=line_style, LineWidth=line_width);
         if show_legend
            h1.DisplayName = '${\omega_x}_{PathFG}$';    
            h2.DisplayName = '${\omega_y}_{PathFG}$';
            h3.DisplayName = '${\omega_z}_{PathFG}$';
         else 
            h1.HandleVisibility = 'off';
            h2.HandleVisibility = 'off';
            h3.HandleVisibility = 'off';
         end
     end

    ylabel({'Angular'; 'Speed [rad/s]'});
    xlim([0, sim_length]);

    subplot(fig5_rows, fig5_cols, 6);
    hold on; 
    % yscale log;
    % set(gca, 'YScale', 'log');  % Convert to log
    % yticks([1e-3 1e-2]);
    % yticklabels({'10^{-3}','10^{-2}'});
    % legend('Position',[0.38 0.116 0.1 0.05], Orientation='horizontal',  FontSize=1*font_size); 
    lgd = legend('Position',[0.67 0.17 0.1 0.05], FontSize=0.7*font_size, Orientation='horizontal', NumColumns=2); 
    lgd.ItemTokenSize = [12, 10];
    disp("control_inputs.Time");
    disp(size(control_inputs.Time));
    disp("governor_time_data");
    disp(size(governor_time_data));
    disp("MPC_time.Data");
    disp(size(MPC_time.Data));
     
    if(ref_source == 1)
        governor_time_data = downsample(governor_time_data, downsample_rate);
    end

    governor_type = "";
    if(ref_source == 1)
        governor_type = "ERG";
    elseif(ref_source == 2)
        governor_type = "ERG";
    elseif(ref_source == 3)
        governor_type = "PathFG";
    else
        governor_type = "Unknown Governor";
    end

    h = plot(control_inputs.Time, governor_time_data + MPC_time.Data, Color=colors(2), LineStyle=line_style, LineWidth=line_width);
    if show_legend
        % h.DisplayName = strcat(governor_type+'+MPC ($N{=}',num2str(N),'$)');
        h.DisplayName = strcat(governor_type+'+MPC');
    else
        h.HandleVisibility = 'off';
    end
    
    h = plot(control_inputs.Time, governor_time_data,  Color=color, LineStyle=line_style, LineWidth=line_width);
    if show_legend
        h.DisplayName = strcat(governor_type);
    else
        h.HandleVisibility = 'off';
    end 
    ytickformat('%.3f');
    ylabel({'Compute Time [s]'});
    xlabel({'', 'Time [s]'})
    xlim([0, sim_length]);


    %%

    % exportation
    for fig_idx = 1:fig_count
        exportgraphics(figure(fig_idx), strcat('plot', num2str(fig_idx), '.pdf'), BackgroundColor='none', ContentType='vector');
        % saveas(figure(fig_idx), strcat('fig', num2str(fig_idx), '.svg'))
        % saving as pdf doesn't support font embedding for non-standard fonts
    end
    % export_fig 'fig5.pdf'

end