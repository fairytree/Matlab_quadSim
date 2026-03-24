function animateTrajectoryMultiSolver(...
    start, goal, obstacles, obstacle_sizes, ...
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
