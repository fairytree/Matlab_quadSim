function animateTrajectoryMultiSolver(...
    start, goal, rect_obs, ...
    states_ts, ref_data_raw, ...
    buffer, ...
    ax, trajColor, refColor, traj_legend_name, ref_legend_name)
% animateTrajectoryMultiSolver  Overlay one solver's trajectory on an
%   existing 2-D axes (unicycle version of the quadrotor helper).
%
%   INPUTS
%     start, goal       [2×1]        start / goal positions
%     rect_obs          [n_o × 4]    rectangular obstacles [x1 y1 x2 y2]
%     states_ts         timeseries   actual states  [T × n_x]
%     ref_data_raw      [T × n_x] or timeseries   reference signal
%     buffer            scalar       safety buffer around obstacles
%     ax                axes handle  target axes (shared across calls)
%     trajColor         color spec   actual trajectory colour
%     refColor          color spec   reference trajectory colour
%     traj_legend_name  string       legend label for actual trajectory
%     ref_legend_name   string       legend label for reference
%
%   Does NOT create a figure or draw obstacles — the caller sets those up
%   once, then calls this function for each solver / N value.

    set(groot, 'defaultTextInterpreter', 'latex');

    pause_time = 0;
    line_width = 3;
    collision_marker = 20;

    %% ---- extract / orient data -----------------------------------------
    x_data = squeeze(states_ts.Data);
    if size(x_data,1) ~= numel(states_ts.Time)
        x_data = x_data';
    end

    % ref_data_raw may be a timeseries or a plain matrix
    if isa(ref_data_raw, 'timeseries')
        ref_data = squeeze(ref_data_raw.Data);
        if size(ref_data,1) ~= numel(ref_data_raw.Time)
            ref_data = ref_data';
        end
    else
        ref_data = ref_data_raw;
    end

    T     = size(x_data, 1);
    n_obs = size(rect_obs, 1);

    %% ---- robot design (differential-drive, same as animateTrajectory) --
    col_body  = [0.9290 0.6940 0.1250];
    col_wheel = [0.3010 0.7450 0.9330];
    body_L = 0.12;  body_W = 0.06;
    wh_L = 0.04;    wh_W = 0.015;
    cast_r = 0.015;

    hg_robot = hgtransform('Parent', ax);

    % chassis
    chassis_x = [-body_L, body_L, body_L, -body_L];
    chassis_y = [-body_W, -body_W, body_W, body_W];
    patch('XData', chassis_x, 'YData', chassis_y, ...
          'FaceColor', col_body, 'EdgeColor','none', 'FaceAlpha',0.9, ...
          'Parent', hg_robot, 'HandleVisibility','off');
    % heading arrow
    plot(ax, [0 body_L*1.1], [0 0], 'k-', 'LineWidth',1.5, ...
         'Parent', hg_robot, 'HandleVisibility','off');
    % left wheel
    lw_x = [-wh_W, wh_W, wh_W, -wh_W];
    lw_y = [body_W, body_W, body_W+wh_L, body_W+wh_L];
    patch('XData', lw_x, 'YData', lw_y, ...
          'FaceColor', col_wheel, 'EdgeColor','k', 'LineWidth',1, ...
          'Parent', hg_robot, 'HandleVisibility','off');
    % right wheel
    rw_y = [-body_W-wh_L, -body_W-wh_L, -body_W, -body_W];
    patch('XData', lw_x, 'YData', rw_y, ...
          'FaceColor', col_wheel, 'EdgeColor','k', 'LineWidth',1, ...
          'Parent', hg_robot, 'HandleVisibility','off');
    % caster
    th = linspace(0, 2*pi, 20);
    patch('XData', body_L*0.7+cast_r*cos(th), 'YData', cast_r*sin(th), ...
          'FaceColor',[0.4 0.4 0.4], 'EdgeColor','k', 'LineWidth',0.5, ...
          'Parent', hg_robot, 'HandleVisibility','off');
    % axle
    plot(ax, [0 0], [-body_W-wh_L, body_W+wh_L], '-', ...
         'Color',[0.3 0.3 0.3], 'LineWidth',1, ...
         'Parent', hg_robot, 'HandleVisibility','off');

    %% ---- animate -------------------------------------------------------
    for i = 1:T
        last_i = max(1, i-1);

        % reference (commented out — uncomment to restore)
%         if i <= size(ref_data,1)
%             plot(ax, ref_data(last_i:i, 1), ref_data(last_i:i, 2), ...
%                  ':', 'LineWidth', line_width, 'Color', refColor, ...
%                  'DisplayName', ref_legend_name);
%         end

        % actual trajectory (solid, caller-chosen colour)
        plot(ax, x_data(last_i:i, 1), x_data(last_i:i, 2), ...
             '-', 'LineWidth', line_width, 'Color', trajColor, ...
             'DisplayName', traj_legend_name);

        % collision markers (actual)
        for j = 1:n_obs
            if pointInBox([x_data(i,1); x_data(i,2)], rect_obs(j,:), buffer)
                plot(ax, x_data(i,1), x_data(i,2), 'ro', ...
                     'MarkerSize', collision_marker, 'HandleVisibility','off');
            end
        end
        % collision markers (reference — commented out with ref lines)
%         if i <= size(ref_data,1)
%             for j = 1:n_obs
%                 if pointInBox([ref_data(i,1); ref_data(i,2)], rect_obs(j,:), buffer)
%                     plot(ax, ref_data(i,1), ref_data(i,2), 'ro', ...
%                          'MarkerSize', collision_marker, 'HandleVisibility','off');
%                 end
%             end
%         end

        % move robot
        heading = getHeading2D(x_data, i);
        T_mat = makehgtform('translate',[x_data(i,1) x_data(i,2) 0]) * ...
                makehgtform('zrotate', heading);
        set(hg_robot, 'Matrix', T_mat);

        drawnow;
        pause(pause_time);
    end
end


%% ========================================================================
%  LOCAL HELPERS
%  ========================================================================

function inside = pointInBox(pt, obs_row, buf)
    inside = pt(1) >= obs_row(1)-buf && pt(1) <= obs_row(3)+buf && ...
             pt(2) >= obs_row(2)-buf && pt(2) <= obs_row(4)+buf;
end

function heading = getHeading2D(x_data, i)
    if size(x_data,2) >= 3
        heading = x_data(i, 3);
    elseif i > 1
        heading = atan2(x_data(i,2)-x_data(i-1,2), x_data(i,1)-x_data(i-1,1));
    else
        heading = 0;
    end
end
