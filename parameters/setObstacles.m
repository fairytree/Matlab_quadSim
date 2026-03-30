
%% Generate obstacles

% options: random positions = 1; Paraboloid trap = 2; 
obstacle_type = 2; 

if (obstacle_type == 1)
    obstacles = [0.9, 1.3, 1.0;
                 1.45, 1.33, 0.4;
                 1.44, 1.33, 0.9;
                 1.44, 1.33, 1.2;
                 1.0, 2.5, 0.8; 
                 1.9, 2.6, 0.8;
                 0.5, 2.0, 0.6;
                 2.5, 1.75, 1.3;
                 0.5, 1.0, 0.4;
                 1.2, 0.45,0.5;
                 1.2, 2.25, 1.5;
                 2.2, 1.3, 1.0];
    obstacle_sizes = [0.1; 0.2; 0.3; 0.3; 0.3; 0.2; 0.2; 0.2; 0.3; 0.4; 0.4; 0.2];
    
elseif(obstacle_type == 2)    
    height = 0.6;            
    radius = 0.8;            
    num_spheres_height = 2;   
    num_spheres_circ = 5;     
    sphere_radius = 0.3;        

    axis_vec = goal - start; 
    axis_dir = axis_vec / norm(axis_vec);
    trap_position = start + 0.3 * axis_vec;  % slightly closer to goal

    % Rotation to align z-axis with axis_vec
    z_axis = [0;0;1];
    v = cross(z_axis, axis_dir);
    s = norm(v);
    c = dot(z_axis, axis_dir);
    if s ~= 0
        vx = [  0    -v(3)  v(2);
               v(3)   0    -v(1);
              -v(2)  v(1)   0 ];
        R = eye(3) + vx + vx^2*((1-c)/s^2); 
    else
        R = eye(3);
    end

    % Generate paraboloid obstacles
    obstacles = [];
    obstacle_sizes = [];
    for i = 1:num_spheres_height
        t = i/num_spheres_height;
        z = t * height;               
        r = sqrt(1 - z/height) * radius; 
        for j = 1:num_spheres_circ
            theta = 2*pi*j/num_spheres_circ;
            local_pos = [r*cos(theta); r*sin(theta); z]; 
            global_pos = trap_position + R*local_pos; 
            obstacles = [obstacles; global_pos'];       
            obstacle_sizes = [obstacle_sizes; sphere_radius]; 
        end
    end

    %     obstacles = [obstacles; 
    %              % 0.9, 1.3, 1.0;
    %              % 1.45, 1.33, 0.4;
    %              % 1.44, 1.33, 0.9;
    %              1.8, 1.8, 1.2;
    %              1.0, 2.5, 0.8; 
    %              1.9, 2.6, 0.8;
    %              % 0.5, 2.0, 0.6;
    %              2.705, 1.75, 0.82;
    %              % 0.5, 1.0, 0.4;
    %              % 1.2, 0.45,0.5;
    %              1.2, 2.25, 1.5;
    %              2.0, 1.3, 0.5];
    % obstacle_sizes = [obstacle_sizes; 0.3; 0.3; 0.2; 0.3; 0.4; 0.2];

    obstacles = [obstacles; 
                 % 0.9, 1.3, 1.0;
                 % 1.45, 1.33, 0.4;
                 % 1.44, 1.33, 0.9;
                 1.8, 1.8, 1.2;
                 1.0, 2.5, 0.8; 
                 1.9, 2.6, 0.8;
                 % 0.5, 2.0, 0.6;
                 2.705, 1.75, 0.82;
                 % 0.5, 1.0, 0.4;
                 % 1.2, 0.45,0.5;
                 1.2, 2.25, 1.5;
                 2.0, 1.3, 0.5;
                 2.6, 0.8, 1.0];
    obstacle_sizes = [obstacle_sizes; 0.3; 0.3; 0.2; 0.3; 0.4; 0.2; 0.2];

end

% %% Visualization
% figure;
% hold on; grid on; axis equal;
% plot3(start(1), start(2), start(3), 'go','MarkerFaceColor','g','MarkerSize',10);
% plot3(goal(1), goal(2), goal(3), 'bo','MarkerFaceColor','b','MarkerSize',10);
% 
% for k = 1:size(obstacles,1)
%     [x_o, y_o, z_o] = sphere(25);  % higher resolution
%     x_o_inner = x_o * obstacle_sizes(k) + obstacles(k,1);
%     y_o_inner = y_o * obstacle_sizes(k) + obstacles(k,2);
%     z_o_inner = z_o * obstacle_sizes(k) + obstacles(k,3);
% 
%     surf(x_o_inner, y_o_inner, z_o_inner, 'EdgeAlpha', 0);  % exactly like animateTrajectory
% end
% 
% xlabel('X'); ylabel('Y'); zlabel('Z');
% view(3); title('Paraboloid Trap (AnimateTrajectory Style)');
