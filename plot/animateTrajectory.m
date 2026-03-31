% animateTrajectory  —  2-D unicycle trajectory animation with collision check.
%
%   animateTrajectory(states_ts, ctrl_inputs_ts, ref_sig_ts, path, rect_obs, buffer, start, goal)
%
%   INPUTS  (Simulink timeseries + static data)
%     states_ts      timeseries   actual system states     [T × n_x]
%     ctrl_inputs_ts timeseries   MPC control inputs       [T × n_u]
%     ref_sig_ts     timeseries   PathFG auxiliary ref      [T × n_x]
%     path           [W × 2]     RRT* waypoints
%     rect_obs       [n_o × 4]   rectangular obstacles  [x1 y1 x2 y2] per row
%     buffer         scalar       safety buffer around each obstacle
%     start          [2 × 1]     start position
%     goal           [2 × 1]     goal  position


function animateTrajectory(states_ts, ctrl_inputs_ts, ref_sig_ts, path, rect_obs, buffer, start, goal, varargin)
    if ~isempty(varargin)
        out_filename = varargin{1};
    else
        out_filename = '';
    end

    %% ---- extract arrays from timeseries --------------------------------
    t_vec = states_ts.Time(:)';             % [1 × T]
    x_data = squeeze(states_ts.Data);       % [T × n_x] or [n_x × T]
    if size(x_data,1) ~= numel(t_vec)
        x_data = x_data';                   % ensure [T × n_x]
    end
    u_data = squeeze(ctrl_inputs_ts.Data);
    if size(u_data,1) ~= size(ctrl_inputs_ts.Time,1)
        u_data = u_data';
    end
    ref_data = squeeze(ref_sig_ts.Data);
    if size(ref_data,1) ~= size(ref_sig_ts.Time,1)
        ref_data = ref_data';
    end
    % Resample reference to the states time grid (ref may be at a slower rate)
    if size(ref_data,1) ~= numel(t_vec)
        ref_time = ref_sig_ts.Time(:)';
        ref_data = interp1(ref_time, ref_data, t_vec', 'previous', 'extrap');
    end

    T     = size(x_data, 1);
    n_obs = size(rect_obs, 1);

    %% ---- tunables ----
    pause_time       = 0;              % 0 = max speed
    downsample_k     = 1;              % plot every k-th step  (1 = all)
    record_video     = false;
    collision_marker = 20;             % large red circle 
    line_width       = 3;              % thick lines 

    %% ---- robot design parameters (differential-drive unicycle) ---------
    %  Two drive wheels on an axle + a small caster wheel in front.
    %    mustard  [0.9290 0.6940 0.1250]  — chassis body
    %    sky blue [0.3010 0.7450 0.9330]  — wheels / accents
    col_body  = [0.9290 0.6940 0.1250];
    col_wheel = [0.3010 0.7450 0.9330];

    body_L = 0.12;   % half-length of chassis rectangle
    body_W = 0.06;   % half-width  of chassis rectangle
    wh_L   = 0.04;   % wheel half-length (along axle direction → appears as height)
    wh_W   = 0.015;  % wheel half-width  (tread thickness)
    cast_r = 0.015;  % caster wheel radius

    set(groot, 'defaultTextInterpreter', 'latex');

    %% ---- frame indices -------------------------------------------------
    idx = 1:downsample_k:T;
    if idx(end) ~= T, idx(end+1) = T; end

    %% ---- figure setup ----
    fig = figure('Name','Unicycle Animation','NumberTitle','off', ...
                 'pos', [0 50 1200 500]);
    ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    xlabel(ax,'$x$ [m]'); ylabel(ax,'$y$ [m]');
    xlim([-2,10]);
    ylim([-1,7]);
    % title(ax, 'Unicycle Trajectory Animation', 'Interpreter','latex');

    %% ---- static elements: obstacles ------------------------------------
    %  Light blue fill, no edge lines.
    obs_color = [0.85 0.92 1.0];   % very light blue (publication-friendly)

    for k = 1:n_obs
        x1 = rect_obs(k,1); y1 = rect_obs(k,2);
        x2 = rect_obs(k,3); y2 = rect_obs(k,4);
        fill(ax, [x1 x2 x2 x1], [y1 y1 y2 y2], ...
             obs_color, 'EdgeColor','none', ...
             'HandleVisibility','off');
    end
    % buffer outlines (red dashed)
    for k = 1:n_obs
        bx1 = rect_obs(k,1)-buffer; by1 = rect_obs(k,2)-buffer;
        bx2 = rect_obs(k,3)+buffer; by2 = rect_obs(k,4)+buffer;
        plot(ax, [bx1 bx2 bx2 bx1 bx1], [by1 by1 by2 by2 by1], ...
             'r--', 'LineWidth', 0.8, 'HandleVisibility','off');
    end

    %% ---- start, goal, path ----
    plot(ax, start(1), start(2), 'go', 'MarkerSize',10, ...
         'MarkerFaceColor','g', 'HandleVisibility','off');
    plot(ax, goal(1),  goal(2),  'bo', 'MarkerSize',10, ...
         'MarkerFaceColor','b', 'HandleVisibility','off');

    %% ---- build unicycle chassis using hgtransform ----
    %  All parts are drawn in the BODY frame (robot at origin, heading = +x)
    %  and moved each frame with a single hgtransform matrix.
    hg_robot = hgtransform('Parent', ax);

    % chassis rectangle (mustard)
    chassis_x = [-body_L, body_L, body_L, -body_L];
    chassis_y = [-body_W, -body_W, body_W, body_W];
    patch('XData', chassis_x, 'YData', chassis_y, ...
          'FaceColor', col_body, 'EdgeColor', 'none', ...
          'FaceAlpha', 0.9, 'Parent', hg_robot, 'HandleVisibility','off');

    % heading arrow (thin dark line from centre to front)
    plot(ax, [0 body_L*1.1], [0 0], 'k-', 'LineWidth', 1.5, ...
         'Parent', hg_robot, 'HandleVisibility','off');

    % left drive wheel (sky blue rectangle)
    lw_x = [-wh_W, wh_W, wh_W, -wh_W];
    lw_y = [body_W, body_W, body_W+wh_L, body_W+wh_L];
    patch('XData', lw_x, 'YData', lw_y, ...
          'FaceColor', col_wheel, 'EdgeColor', 'k', 'LineWidth', 1, ...
          'Parent', hg_robot, 'HandleVisibility','off');

    % right drive wheel (sky blue rectangle)
    rw_y = [-body_W-wh_L, -body_W-wh_L, -body_W, -body_W];
    patch('XData', lw_x, 'YData', rw_y, ...
          'FaceColor', col_wheel, 'EdgeColor', 'k', 'LineWidth', 1, ...
          'Parent', hg_robot, 'HandleVisibility','off');

    % caster wheel (small filled circle at front)
    th = linspace(0, 2*pi, 20);
    patch('XData', body_L*0.7 + cast_r*cos(th), ...
          'YData', cast_r*sin(th), ...
          'FaceColor', [0.4 0.4 0.4], 'EdgeColor','k', 'LineWidth', 0.5, ...
          'Parent', hg_robot, 'HandleVisibility','off');

    % axle line (thin line connecting the two wheels)
    plot(ax, [0 0], [-body_W-wh_L, body_W+wh_L], '-', ...
         'Color', [0.3 0.3 0.3], 'LineWidth', 1, ...
         'Parent', hg_robot, 'HandleVisibility','off');

    %% ---- video writer (optional) ----------------------------------------
    if record_video
        vid = VideoWriter('animateTrajectory', 'Motion JPEG AVI');
        vid.FrameRate = 30;
        open(vid);
    end

    %% ---- collision flag ------------------------------------------------
    collision_found = false;

    %% ---- animation loop ----
    for frame = 1:numel(idx)
        i = idx(frame);
        last_i = idx(max(1, frame-1));

        px = x_data(i, 1);
        py = x_data(i, 2);

        % ---- grow reference line (commented out — uncomment to restore) --
%         if i <= size(ref_data,1)
%             ri = min(i, size(ref_data,1));
%             rli = min(last_i, size(ref_data,1));
%             h1 = plot(ax, ref_data(rli:ri, 1), ref_data(rli:ri, 2), ...
%                       'r:', 'LineWidth', line_width, ...
%                       'DisplayName', 'Auxiliary Reference');
%         end
%         % collision check on reference
%         if i <= size(ref_data,1)
%             ref_pt = [ref_data(i,1); ref_data(i,2)];
%             for j = 1:n_obs
%                 if pointInsideInflatedBox(ref_pt, rect_obs(j,:), buffer)
%                     plot(ax, ref_pt(1), ref_pt(2), 'ro', ...
%                          'MarkerSize', collision_marker, ...
%                          'HandleVisibility','off');
%                 end
%             end
%         end

        % ---- grow actual trajectory ----
        h2 = plot(ax, x_data(last_i:i, 1), x_data(last_i:i, 2), ...
                  'b-', 'LineWidth', line_width, ...
                  'DisplayName', 'Actual Trajectory');

        % collision check on actual trajectory
        traj_pt = [px; py];
        for j = 1:n_obs
            if pointInsideInflatedBox(traj_pt, rect_obs(j,:), buffer)
                plot(ax, px, py, 'ro', ...
                     'MarkerSize', collision_marker, ...
                     'HandleVisibility','off');
                if ~collision_found
                    fprintf("  *** COLLISION (actual) at t = %.2f s, " + ...
                            "pos = [%.3f, %.3f], obs #%d\n", ...
                            t_vec(i), px, py, j);
                    collision_found = true;
                end
            end
        end

        % ---- move unicycle chassis ----
        heading = getHeading(x_data, u_data, i);
        T_mat = makehgtform('translate', [px py 0]) * ...
                makehgtform('zrotate', heading);
        set(hg_robot, 'Matrix', T_mat);

        % ---- draw & record ----------------------------------------------
        drawnow;
        if record_video
            writeVideo(vid, getframe(fig));
        end
        pause(pause_time);
    end

    %% ---- cleanup -------------------------------------------------------
    if record_video, close(vid); end

    if ~collision_found
        disp("  No collisions detected.");
    end

    %% ---- legend (once, after animation) ----
    hLines = findobj(ax, 'Type','line');
    hLines = flip(hLines);
    [~, ui] = unique({hLines.DisplayName}, 'stable');
    lgd = legend(ax, hLines(ui), 'Position', [0.6 0.73 0.2 0.15]);
    lgd.Box = 'off';
    lgd.FontSize = 12;

    exportgraphics(fig, '2dTraj.pdf', 'ContentType','vector');

    if ~isempty(out_filename)
        exportgraphics(fig, out_filename, 'ContentType','vector', 'BackgroundColor','none');
        fprintf("Exported %s\n", out_filename);
    end
end


%% ========================================================================
%  LOCAL HELPERS
%  ========================================================================

function inside = pointInsideInflatedBox(pt, obs_row, buf)
    inside = pt(1) >= obs_row(1) - buf && pt(1) <= obs_row(3) + buf && ...
             pt(2) >= obs_row(2) - buf && pt(2) <= obs_row(4) + buf;
end


function heading = getHeading(x_data, u_data, i)
%getHeading  Extract or estimate unicycle heading at step i.
    if size(x_data,2) >= 3
        heading = x_data(i, 3);          % theta stored as state 3
    elseif i <= size(u_data,1)
        heading = u_data(i, 1);          % fallback: steering input
    elseif i > 1
        heading = atan2(x_data(i,2)-x_data(i-1,2), ...
                        x_data(i,1)-x_data(i-1,1));
    else
        heading = 0;
    end
end
