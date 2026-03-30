function animateTrajectoryMultiSolver(...
    start, goal, obstacles, obstacle_sizes, rect_obs, ...
    non_linear_full_states, reference_signal, ...
    agent_size, sample_time_continous, sample_time_pathFG, ...
    ax, trajColor, refColor, traj_legend_name, ref_legend_name)

    pause_time = 0;
    display_time = false;

    set(groot, 'defaultTextInterpreter', 'latex')

    %% Downsample states
    downsample_scalar_trajectory = 100;
    time = downsample(non_linear_full_states.Time, downsample_scalar_trajectory);
    x = downsample(non_linear_full_states.Data(:,1)', downsample_scalar_trajectory);
    y = downsample(non_linear_full_states.Data(:,2)', downsample_scalar_trajectory);
    z = downsample(non_linear_full_states.Data(:,3)', downsample_scalar_trajectory);
    roll = downsample(non_linear_full_states.Data(:,7)', downsample_scalar_trajectory);
    pitch = downsample(non_linear_full_states.Data(:,8)', downsample_scalar_trajectory);
    yaw = downsample(non_linear_full_states.Data(:,9)', downsample_scalar_trajectory);

    %% Downsample reference signal
    downsample_scalar_ref = downsample_scalar_trajectory * sample_time_continous / sample_time_pathFG;
    ref_x = downsample(reference_signal(:,1)', downsample_scalar_ref);
    ref_y = downsample(reference_signal(:,2)', downsample_scalar_ref);
    ref_z = downsample(reference_signal(:,3)', downsample_scalar_ref);

    %% Drone design parameters
    D2R = pi/180;
    b   = 0.3;   a = b/4;
    H   = 0.02;  H_m = H+H; r_p = b/5;

    %% Base rotation
    ro = 45*D2R;
    Ri = [cos(ro) -sin(ro) 0;
          sin(ro)  cos(ro) 0;
          0        0       1];
    base_co = [-a/2  a/2 a/2 -a/2; -a/2 -a/2 a/2 a/2; 0 0 0 0];
    base = Ri*base_co;
    to = linspace(0, 2*pi);
    xp = r_p*cos(to); yp = r_p*sin(to); zp = zeros(1,length(to));

    %% Create drone in axes
    hg = ax;
    hold(hg,'on');


    %% Plot obstacles with rainbow color based on z
    for i = 1:size(obstacles,2)
        [x_o, y_o, z_o] = sphere(25);
        x_o_inner = x_o * obstacle_sizes(i) + obstacles(1,i);
        y_o_inner = y_o * obstacle_sizes(i) + obstacles(2,i);
        z_o_inner = z_o * obstacle_sizes(i) + obstacles(3,i);

        surf(ax, x_o_inner, y_o_inner, z_o_inner, z_o_inner, ...
            'EdgeAlpha', 0, 'FaceColor', 'interp');
        hold(ax,'on');
    end

    % Plot rectangular prism obstacles (rainbow-by-Z, same as spheres)
    if exist('rect_obs', 'var') && ~isempty(rect_obs)
        n_subdiv = 20;
        for k = 1:size(rect_obs, 1)
            x1 = rect_obs(k, 1); y1 = rect_obs(k, 2); z1 = rect_obs(k, 3);
            x2 = rect_obs(k, 4); y2 = rect_obs(k, 5); z2 = rect_obs(k, 6);
            z_vec = linspace(z1, z2, n_subdiv)';
            % Bottom face
            surf([x1 x2; x1 x2], [y1 y1; y2 y2], [z1 z1; z1 z1], 'EdgeAlpha', 0);
            % Top face
            surf([x1 x2; x1 x2], [y1 y1; y2 y2], [z2 z2; z2 z2], 'EdgeAlpha', 0);
            % Front face (y = y1)
            surf(repmat([x1 x2], n_subdiv, 1), repmat([y1 y1], n_subdiv, 1), repmat(z_vec, 1, 2), 'EdgeAlpha', 0);
            % Back face (y = y2)
            surf(repmat([x1 x2], n_subdiv, 1), repmat([y2 y2], n_subdiv, 1), repmat(z_vec, 1, 2), 'EdgeAlpha', 0);
            % Left face (x = x1)
            surf(repmat([x1 x1], n_subdiv, 1), repmat([y1 y2], n_subdiv, 1), repmat(z_vec, 1, 2), 'EdgeAlpha', 0);
            % Right face (x = x2)
            surf(repmat([x2 x2], n_subdiv, 1), repmat([y1 y2], n_subdiv, 1), repmat(z_vec, 1, 2), 'EdgeAlpha', 0);
        end
    end
    
    %% Drone parts
    drone(1) = patch(base(1,:), base(2,:), base(3,:), [0.9290 0.6940 0.1250], 'EdgeColor','none');
    drone(2) = patch(base(1,:), base(2,:), base(3,:)+H, [0.9290 0.6940 0.1250], 'EdgeColor','none');
    [xc, yc, zc] = cylinder([H/2 H/2]);
    drone(3) = surface(b*zc-b/2, yc, xc+H/2,'facecolor',[0.3010 0.7450 0.9330],'EdgeColor','none');
    drone(4) = surface(yc, b*zc-b/2, xc+H/2,'facecolor',[0.9290 0.6940 0.1250],'EdgeColor','none');
    drone(5) = surface(xc+b/2, yc, H_m*zc+H/2,'facecolor',[0.3010 0.7450 0.9330],'EdgeColor','none');
    drone(6) = surface(xc-b/2, yc, H_m*zc+H/2,'facecolor',[0.3010 0.7450 0.9330],'EdgeColor','none');
    drone(7) = surface(xc, yc+b/2, H_m*zc+H/2,'facecolor',[0.9290 0.6940 0.1250],'EdgeColor','none');
    drone(8) = surface(xc, yc-b/2, H_m*zc+H/2,'facecolor',[0.9290 0.6940 0.1250],'EdgeColor','none');
    drone(9) = patch(xp+b/2, yp, zp+(H_m+H/2), [0.3010 0.7450 0.9330], 'LineWidth',0.5,'EdgeColor','none');
    drone(10) = patch(xp-b/2, yp, zp+(H_m+H/2), [0.3010 0.7450 0.9330], 'LineWidth',0.5,'EdgeColor','none');
    drone(11) = patch(xp, yp+b/2, zp+(H_m+H/2), [0.9290 0.6940 0.1250], 'LineWidth',0.5,'EdgeColor','none');
    drone(12) = patch(xp, yp-b/2, zp+(H_m+H/2), [0.9290 0.6940 0.1250], 'LineWidth',0.5,'EdgeColor','none');

    %% Group object for drone
    combinedobject = hgtransform('parent',hg);
    set(drone,'Parent',combinedobject);

    %% Animate trajectory
    for i = 1:length(x)
        last_i = max([1, i-1]);

        % Plot reference trajectory
        h1 = plot3(ref_x(last_i:i), ref_y(last_i:i), ref_z(last_i:i), ':', ...
                  'LineWidth',3, 'DisplayName',ref_legend_name, 'Color', refColor);

        % Plot actual trajectory
        h2 = plot3(x(last_i:i), y(last_i:i), z(last_i:i), '-', ...
                  'LineWidth',3, 'DisplayName',traj_legend_name, 'Color', trajColor);

        
        % % reference
        % h1 = plot3(ax, ref_x(last_i:i), ref_y(last_i:i), ref_z(last_i:i), [refColor,':'],'LineWidth',3,'DisplayName','Aux Reference');
        % % trajectory
        % h2 = plot3(ax, x(last_i:i), y(last_i:i), z(last_i:i), [trajColor,'-'],'LineWidth',3,'DisplayName','Actual Trajectory');

        % collision markers
        for j = 1:size(obstacles,2)
            if norm([ref_x(i); ref_y(i); ref_z(i)] - obstacles(:,j)) <= agent_size + obstacle_sizes(j)
                plot3(ax, ref_x(i), ref_y(i), ref_z(i), 'ro','MarkerSize',20);
            end
            if norm([x(i); y(i); z(i)] - obstacles(:,j)) <= agent_size + obstacle_sizes(j)
                plot3(ax, x(i), y(i), z(i), 'ro','MarkerSize',20);
            end
        end

        % box obstacle collision check for reference
        if ~isempty(rect_obs)
            ref_pt = [ref_x(i); ref_y(i); ref_z(i)];
            for j = 1:size(rect_obs, 1)
                box_min = rect_obs(j, 1:3)' - agent_size;
                box_max = rect_obs(j, 4:6)' + agent_size;
                if all(ref_pt >= box_min) && all(ref_pt <= box_max)
                    plot3(ref_x(i), ref_y(i), ref_z(i), "ro", "MarkerSize", 20);
                end
            end
        end

        % box obstacle collision check for actual trajectory
        if ~isempty(rect_obs)
            traj_pt = [x(i); y(i); z(i)];
            for j = 1:size(rect_obs, 1)
                box_min = rect_obs(j, 1:3)' - agent_size;
                box_max = rect_obs(j, 4:6)' + agent_size;
                if all(traj_pt >= box_min) && all(traj_pt <= box_max)
                    plot3(x(i), y(i), z(i), "ro", "MarkerSize", 20);
                end
            end
        end

        % drone transformation
        translation = makehgtform('translate',[x(i) y(i) z(i)]);
        rotation1 = makehgtform('xrotate',roll(i));
        rotation2 = makehgtform('yrotate',pitch(i));
        rotation3 = makehgtform('zrotate',yaw(i)-pi/4);
        set(combinedobject,'Matrix',translation*rotation3*rotation2*rotation1);

        drawnow;
        pause(pause_time);
    end
end
