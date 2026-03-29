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
%
%   The animation draws:
%     – grey-filled obstacles and red-dashed buffer regions
%     – RRT* path, start & goal markers
%     – actual trajectory (blue) growing each frame
%     – auxiliary reference trajectory (magenta dashed) growing each frame
%     – a unicycle triangle marker at the current position & heading
%     – RED CIRCLE markers at any frame where the actual position
%       lies inside an inflated obstacle (collision)

function animateTrajectory(states_ts, ctrl_inputs_ts, ref_sig_ts, path, rect_obs, buffer, start, goal)

    %% ---- extract arrays from timeseries --------------------------------
    t_vec = states_ts.Time(:)';             % [1 × T]
    x_data = squeeze(states_ts.Data);       % [T × n_x] or [n_x × T]
    if size(x_data,1) ~= numel(t_vec)
        x_data = x_data';                   % ensure [T × n_x]
    end
    % steering angle from ctrl_inputs (col 1) used for heading
    u_data = squeeze(ctrl_inputs_ts.Data);
    if size(u_data,1) ~= size(ctrl_inputs_ts.Time,1)
        u_data = u_data';
    end
    % auxiliary reference from PathFG
    ref_data = squeeze(ref_sig_ts.Data);
    if size(ref_data,1) ~= size(ref_sig_ts.Time,1)
        ref_data = ref_data';
    end

    T     = size(x_data, 1);
    n_obs = size(rect_obs, 1);

    %% ---- tunables ------------------------------------------------------
    pause_time    = 0.01;           % pause between frames (s), 0 = max speed
    downsample_k  = 1;              % plot every k-th step  (1 = all)
    record_video  = false;          % set true to write .avi
    marker_size   = 15;             % size of collision marker
    unicycle_len  = 0.15;           % half-length of the triangle marker
    line_width    = 2;

    set(groot, 'defaultTextInterpreter', 'latex');

    %% ---- frame indices -------------------------------------------------
    idx = 1:downsample_k:T;
    if idx(end) ~= T, idx(end+1) = T; end

    %% ---- figure setup --------------------------------------------------
    fig = figure('Name','Unicycle Animation','NumberTitle','off', ...
                 'Units','normalized','Position',[0.1 0.1 0.6 0.7]);
    ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    xlabel(ax,'$x$ [m]'); ylabel(ax,'$y$ [m]');
    title(ax, 'Unicycle Trajectory Animation', 'Interpreter','latex');

    % --- static elements: obstacles, buffer, path, start/goal ------------
    for k = 1:n_obs
        x1 = rect_obs(k,1); y1 = rect_obs(k,2);
        x2 = rect_obs(k,3); y2 = rect_obs(k,4);
        fill(ax, [x1 x2 x2 x1], [y1 y1 y2 y2], ...
             [0.85 0.85 0.85], 'EdgeColor','k', 'HandleVisibility','off');
    end
    for k = 1:n_obs
        bx1 = rect_obs(k,1)-buffer; by1 = rect_obs(k,2)-buffer;
        bx2 = rect_obs(k,3)+buffer; by2 = rect_obs(k,4)+buffer;
        plot(ax, [bx1 bx2 bx2 bx1 bx1], [by1 by1 by2 by2 by1], ...
             'r--', 'HandleVisibility','off');
    end
    plot(ax, path(:,1), path(:,2), 'b--', 'LineWidth',1, 'DisplayName','RRT* Path');
    plot(ax, start(1), start(2), 'go', 'MarkerSize',12, ...
         'MarkerFaceColor','g', 'DisplayName','Start');
    plot(ax, goal(1), goal(2), 'rp', 'MarkerSize',15, ...
         'MarkerFaceColor','r', 'DisplayName','Goal');

    % --- animated line objects -------------------------------------------
    h_traj = animatedline(ax, 'Color','b', 'LineWidth',line_width, ...
                          'DisplayName','Actual');
    h_ref  = animatedline(ax, 'Color','m', 'LineStyle','--', 'LineWidth',line_width, ...
                          'DisplayName','Aux. Ref.');

    % unicycle marker (triangle pointing in heading direction)
    h_body = fill(ax, nan, nan, [0.3 0.75 0.93], 'EdgeColor','k', ...
                  'LineWidth',1.5, 'HandleVisibility','off');

    % time annotation
    h_time = text(ax, 0.02, 0.97, '', 'Units','normalized', ...
                  'FontSize',12, 'VerticalAlignment','top');

    legend(ax, 'Location','best');

    %% ---- video writer (optional) ----------------------------------------
    if record_video
        vid = VideoWriter('animateTrajectory', 'Motion JPEG AVI');
        vid.FrameRate = 30;
        open(vid);
    end

    %% ---- collision flag ------------------------------------------------
    collision_found = false;

    %% ---- animation loop ------------------------------------------------
    for frame = 1:numel(idx)
        i = idx(frame);

        px = x_data(i, 1);
        py = x_data(i, 2);

        % ---- grow trajectory lines --------------------------------------
        addpoints(h_traj, px, py);
        if i <= size(ref_data,1)
            addpoints(h_ref, ref_data(i,1), ref_data(i,2));
        end

        % ---- unicycle triangle ------------------------------------------
        %  heading from steering angle (ctrl_inputs col 1)
        if i <= size(u_data,1)
            heading = u_data(i, 1);
        elseif i > 1
            heading = atan2(x_data(i,2)-x_data(i-1,2), x_data(i,1)-x_data(i-1,1));
        else
            heading = 0;
        end
        tri = unicycleTriangle(px, py, heading, unicycle_len);
        set(h_body, 'XData', tri(1,:), 'YData', tri(2,:));

        % ---- collision check: actual trajectory -------------------------
        traj_pt = [px; py];
        for j = 1:n_obs
            if pointInsideInflatedBox(traj_pt, rect_obs(j,:), buffer)
                plot(ax, px, py, 'ro', ...
                     'MarkerSize', marker_size, 'LineWidth', 2, ...
                     'HandleVisibility','off');
                if ~collision_found
                    fprintf("  *** COLLISION (actual) at t = %.2f s, " + ...
                            "pos = [%.3f, %.3f], obs #%d\n", ...
                            t_vec(i), px, py, j);
                    collision_found = true;
                end
            end
        end

        % ---- collision check: auxiliary reference -----------------------
        if i <= size(ref_data,1)
            ref_pt = [ref_data(i,1); ref_data(i,2)];
            for j = 1:n_obs
                if pointInsideInflatedBox(ref_pt, rect_obs(j,:), buffer)
                    plot(ax, ref_pt(1), ref_pt(2), 'ms', ...
                         'MarkerSize', marker_size, 'LineWidth', 2, ...
                         'HandleVisibility','off');
                end
            end
        end

        % ---- time stamp -------------------------------------------------
        set(h_time, 'String', sprintf('$t = %.2f$ s', t_vec(i)));

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

    exportgraphics(fig, 'animateTrajectory.pdf', ...
                   'ContentType','vector', 'BackgroundColor','none');
    fprintf("Exported animateTrajectory.pdf\n");
end


%% ========================================================================
%  LOCAL HELPERS
%  ========================================================================

function inside = pointInsideInflatedBox(pt, obs_row, buf)
    inside = pt(1) >= obs_row(1) - buf && pt(1) <= obs_row(3) + buf && ...
             pt(2) >= obs_row(2) - buf && pt(2) <= obs_row(4) + buf;
end


function tri = unicycleTriangle(cx, cy, theta, L)
    pts_body = [ L,  -L/2,  -L/2;
                 0,   L/2,  -L/2];
    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];
    tri = R * pts_body + [cx; cy];
end
