% Animation for QuadCopter.
function animateTrajectory(...
    start,...
    goal,...
    obstacles,...
    obstacle_sizes,...
    non_linear_full_states,...
    reference_signal,...
    agent_size, ...
    sample_time_continous,...
    sample_time_pathFG)

    pause_time = 0;
    display_time = false;

    set(groot, defaultTextInterpreter = 'latex')
   
    recordVideo = true;   % easy on/off switch


    %% downsample states
    downsample_scalar_trajectory = 100;
    time = downsample(non_linear_full_states.Time, downsample_scalar_trajectory);
    x = downsample(non_linear_full_states.Data(:, 1)', downsample_scalar_trajectory);
    y = downsample(non_linear_full_states.Data(:, 2)', downsample_scalar_trajectory);
    z = downsample(non_linear_full_states.Data(:, 3)', downsample_scalar_trajectory);
    roll = downsample(non_linear_full_states.Data(:, 7)', downsample_scalar_trajectory);
    pitch = downsample(non_linear_full_states.Data(:, 8)', downsample_scalar_trajectory);
    yaw = downsample(non_linear_full_states.Data(:, 9)', downsample_scalar_trajectory);

    
    %% downsample reference signal
    downsample_scalar_ref = downsample_scalar_trajectory * sample_time_continous / sample_time_pathFG;
    ref_x = downsample(reference_signal.Data(:, 1)', downsample_scalar_ref);
    ref_y = downsample(reference_signal.Data(:, 2)', downsample_scalar_ref);
    ref_z = downsample(reference_signal.Data(:, 3)', downsample_scalar_ref);

    % %% Resample reference to match state time grid
    % ref_x = interp1(reference_signal.Time, reference_signal.Data(:,1), time, 'previous', 'extrap');
    % ref_y = interp1(reference_signal.Time, reference_signal.Data(:,2), time, 'previous', 'extrap');
    % ref_z = interp1(reference_signal.Time, reference_signal.Data(:,3), time, 'previous', 'extrap');

    
    %% Define vehicle design parameters
    D2R = pi/180;
    % R2D = 180/pi;
    b   = 0.3;   % the length of total square cover by whole body of quadcopter in meter
    a   = b/4;   % the legth of small square base of quadcopter
    H   = 0.02;  % hight of drone in Z direction (m)
    H_m = H+H; % hight of motor in z direction (m)
    r_p = b/5;   % radius of propeller
  
    
    %% Conversions
    ro = 45*D2R;                   % angle by which rotate the base of quadcopter
    Ri = [cos(ro) -sin(ro) 0;
          sin(ro) cos(ro)  0;
           0       0       1];     % rotation matrix to rotate the coordinates of base 
    base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
               -a/2 -a/2 a/2 a/2;
                 0    0   0   0];
    base = Ri*base_co;             % rotate base Coordinates by 45 degree 
    to = linspace(0, 2*pi);
    xp = r_p*cos(to);
    yp = r_p*sin(to);
    zp = zeros(1,length(to));
   
   
    %% Define Figure plot
    fig2 = figure('pos', [0 50 1200 800]);
    hg = gca;
    hg.Projection = "perspective";

    %% Video recording setup
    if recordVideo
        myWriter = VideoWriter('animateTrajectory', 'Motion JPEG AVI');
        myWriter.FrameRate = 30;
        open(myWriter);
    end
    
    %RRT 3D view
    % view(-1.279217130069161e+02,19.285823935221515); 
    % view(1.2427,0.28353)
    % campos([-6.4099, -17.1034, 13.741])
    view(-7.573304666576624,19.585200472715634);
    % campos(ax1, [-1.412012197832373,-20.40253712848401,8.86123732595924]);
    
    % % RRT 2D: view: -90,90; campos: comment this line
    % view(-90,90);

    % %Potential Field 3D: view: ; campos: -16.89939180776057,12.33975978038676,10.693399249387328
    % view(-1.205039062500000e+02,45.6044921875); 
    % campos([-16.89939180776057,12.33975978038676,10.693399249387328]);


    % % Potential Field 2D: view: -90,90; campos: comment this line
    % view(-90,90);

    grid on;
    axis equal;
    % xlim([start(1,1)-1 goal(1,1)+3]); ylim([start(2,1)-1 goal(2,1)+3]); zlim([start(3,1)-1 goal(3,1)+3]);
    xlim([-1 3.5]); ylim([-1 4.5]); zlim([-1 2.2]);
    xlim([0 3]); ylim([0 3]); zlim([0 2]);
   
    % title('Drone Trajectory')
    xlabel('$x$ [m]');
    ylabel('$y$ [m]');
    zlabel('$z$ [m]');
    hold(gca, 'on');
  
    % draw the start and goal
    plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 10);
    plot3(goal(1), goal(2), goal(3), 'bo', 'MarkerSize', 10);

    % Plot obstacles with their respective sizes
    for i = 1:size(obstacles, 2)
        [x_o, y_o, z_o] = sphere(25);
        x_o_inner = x_o * obstacle_sizes(i) + obstacles(1, i);
        y_o_inner = y_o * obstacle_sizes(i) + obstacles(2, i);
        z_o_inner = z_o * obstacle_sizes(i) + obstacles(3, i);

        % x_o_outer = x_o * (obstacle_sizes(i) + influence_margins(i)) + obstacles(1, i);
        % y_o_outer = y_o * (obstacle_sizes(i) + influence_margins(i)) + obstacles(2, i);
        % z_o_outer = z_o * (obstacle_sizes(i) + influence_margins(i)) + obstacles(3, i);

        surf(x_o_inner, y_o_inner, z_o_inner, 'EdgeAlpha', 0);
        % surf(x_o_outer, y_o_outer, z_o_outer, "EdgeAlpha", 0);
        % surf(x_o_outer, y_o_outer, z_o_outer, "EdgeAlpha", 0, "FaceAlpha", 0.5); % transparent is very computational expensive

        hold on;
    end
   

    %% Design Vehicle Different parts
    % design the base square
    drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],[0.9290 0.6940 0.1250], 'EdgeColor','none');
    drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],[0.9290 0.6940 0.1250], 'EdgeColor','none');
    % alpha(drone(1:2),0.7);
    % design 2 parpendiculer legs of quadcopter 
    [xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);
    drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor',[0.3010 0.7450 0.9330], 'EdgeColor','none');
    drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor',[0.9290 0.6940 0.1250], 'EdgeColor','none');
    % drone(13) =  surface((ycylinder+zcylinder)/6,(ycylinder+zcylinder)/6,xcylinder+H/2,'facecolor','r', 'EdgeColor','none'); 
    % alpha(drone(3:4),0.6);
    % design 4 cylindrical motors 
    drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor',[0.3010 0.7450 0.9330], 'EdgeColor','none');
    drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor',[0.3010 0.7450 0.9330], 'EdgeColor','none');
    drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor',[0.9290 0.6940 0.1250], 'EdgeColor','none');
    drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor',[0.9290 0.6940 0.1250], 'EdgeColor','none');
    % alpha(drone(5:8),0.7);
    % design 4 propellers
    drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),[0.3010 0.7450 0.9330],'LineWidth',0.5, 'EdgeColor','none');
    drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),[0.3010 0.7450 0.9330],'LineWidth',0.5, 'EdgeColor','none');
    drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),[0.9290 0.6940 0.1250],'LineWidth',0.5, 'EdgeColor','none');
    drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),[0.9290 0.6940 0.1250],'LineWidth',0.5, 'EdgeColor','none');
    % alpha(drone(9:12),0.3);
  
    
    %% create a group object and parent surface
    combinedobject = hgtransform('parent',hg );
    set(drone,'parent',combinedobject)


    disp("length_x");
    disp(length(x));
    disp("length_ref");
    disp(length(ref_x));

    %% Draw traj, ref and collision points
    figure(fig2);
    hold on;

    for i = 1:length(x)
        last_i = max([1, i - 1]);
       
        % plot reference
        h1 = plot3(ref_x(last_i:i), ref_y(last_i:i), ref_z(last_i:i), 'r:','LineWidth',3, 'DisplayName', 'Auxiliary Reference');         
        for j = 1:size(obstacles, 2)
            if norm([ref_x(i); ref_y(i); ref_z(i)] - obstacles(:,j)) <= agent_size + obstacle_sizes(j)
                plot3(ref_x(i), ref_y(i), ref_z(i), "ro", "MarkerSize", 20);
            end
        end
        
        % plot actual traj.
        h2 = plot3(x(last_i:i),y(last_i:i),z(last_i:i), 'b-','LineWidth',3, 'DisplayName', 'Actual Trajectory'); 
        for j = 1:size(obstacles, 2)
            if norm([x(i); y(i); z(i)] - obstacles(:,j)) <= agent_size + obstacle_sizes(j)
                plot3(x(i), y(i), z(i), "ro", "MarkerSize", 20);
            end
        end
        legend([h1, h2],'Position', [0.65, 0.75, 0.2, 0.1]);

        % plot time stamp
        if display_time
            timestamp = sprintf('Elapsed Time: %.2f s', time(i));
            if i > 1
                 set(time_text, 'String', timestamp);
            else
                time_text = text(-0.5,-0.5,3.0, timestamp, 'FontSize', 20, 'HorizontalAlignment', 'right');
            end
        end
        
        % plot quadrotor
        translation = makehgtform('translate', [x(i) y(i) z(i)]);
        rotation1 = makehgtform('xrotate',roll(i));
        rotation2 = makehgtform('yrotate',pitch(i));
        rotation3 = makehgtform('zrotate',yaw(i) - pi / 4);
        % scaling = makehgtform('scale',1-i/20);
        set(combinedobject,'matrix',...
            translation*rotation3*rotation2*rotation1);        
        %movieVector(i) =  getframe(fig1);
        %delete(b);
        drawnow;

        if recordVideo
            frame = getframe(fig2);   % capture current figure
            writeVideo(myWriter, frame);
        end


        pause(pause_time);
    end

    exportgraphics(gcf, '3dTraj.pdf', ContentType='vector');

    %% Post-simulation rotation (horizontal orbit)
    if recordVideo
        nFrames = 360;             % number of frames for full rotation
        angleStep = 360 / nFrames; % degrees per frame
        pauseTime = 0.03;          % controls rotation speed
    
        for k = 1:nFrames
            camorbit(hg, angleStep, 0, 'data', [0 0 1]);  % rotate around Z-axis
            drawnow limitrate;
    
            % Optional: record this rotation into video
            if exist('myWriter','var') && isvalid(myWriter)
                frame = getframe(fig2);
                writeVideo(myWriter, frame);
            end
            pause(pauseTime);
        end
    end

    if recordVideo
        close(myWriter);
    end


% % optional: Save the movie
% myWriter = VideoWriter('animateTrajectory', 'Motion JPEG AVI');
% myWriter = VideoWriter('animateTrajectory1', 'MPEG-4');
% myWriter.Quality = 100;
% myWritter.FrameRate = 120;
% 
% % Open the VideoWriter object, write the movie, and class the file
% open(myWriter);
% writeVideo(myWriter, movieVector);
% close(myWriter);



end
