function  [normalized_current_ref, next_ref_idx] = navigationField( ...
    applied_reference, ...
    goal, ...
    obstacles, ...
    obstacle_sizes, ...
    agent_size, ...
    influence_margins,...
    static_saftey_margins, ...
    circulation_gains,...
    smoothing_radius, ...
    goal_threshold, ...
    navigation_field_repulsion_inclusion, ...
    current_ref_idx)

    next_ref_idx = current_ref_idx;
    applied_reference_x_y_z = applied_reference(1:3);
    
    % check if goal reached
    if(norm(applied_reference_x_y_z - goal)) < goal_threshold
        normalized_current_ref = [0; 0; 0];
        return;
    end

    number_of_obstacles = numel(obstacles(1,:));
 
    repulsion_field_obstacles_nonconservative = [0; 0; 0];
    repulsion_field_obstacles_conservative = [0; 0; 0];

    % add up each repulsion field for every indiviual obstacle
    for j = 1:number_of_obstacles
        
        % calculate conservative repulsion field for obstacles
        repulsion_field_obstacles_conservative = repulsion_field_obstacles_conservative +...
            max((influence_margins(j) - distanceFromObstacle(obstacles(:,j),...
            applied_reference_x_y_z, obstacle_sizes(j), agent_size)) /...
            (influence_margins(j) - static_saftey_margins(j)), 0)...
            * normalize_(applied_reference_x_y_z - obstacles(:,j), 0);


        % calculate nonconservative repulsion field for obstacles
        obstacle_position = obstacles(:,j);
        obstacle_size = obstacle_sizes(j);
        distance_from_obstacle = distanceFromObstacle(obstacle_position, applied_reference_x_y_z, obstacle_size, agent_size);

        if influence_margins(j) >= distance_from_obstacle

            repulsion_field_obstacles_nonconservative = repulsion_field_obstacles_nonconservative + circulation_gains(j) * normalize_(...
            [-obstacle_position(2) + applied_reference_x_y_z(2) + obstacle_position(3) - applied_reference_x_y_z(3);...
            obstacle_position(1) - applied_reference_x_y_z(1) - obstacle_position(3) + applied_reference_x_y_z(3);...
            -obstacle_position(1) + applied_reference_x_y_z(1) + obstacle_position(2) - applied_reference_x_y_z(2)], 0);

        end

    end

    % calculate total repulsion field for obstacles
    repulsion_field_obstacles = repulsion_field_obstacles_conservative + repulsion_field_obstacles_nonconservative;

    % disp("repulsion_field_obstacles");
    % disp(repulsion_field_obstacles);
    
    % calculate attraction and repulsion fields
    attraction_field = normalize_(goal - applied_reference_x_y_z, smoothing_radius);
    
    if navigation_field_repulsion_inclusion
        repulsion_field = repulsion_field_obstacles;
    else
        repulsion_field = 0;
    end

    % return the total navigation field
    normalized_current_ref = attraction_field + repulsion_field;

    
end


function normalized_vector = normalize_(vector, smoothing_radius)

    normalized_vector = vector / max(norm(vector), smoothing_radius);

end


function distance_from_obstacle = distanceFromObstacle(obstacle_position, agent_position, obstacle_size, agent_size)

    distance_from_obstacle = norm(agent_position - obstacle_position) - agent_size - obstacle_size;

end

% 
% % Example: 
% % define variables
% goal = gpuArray([5; 5; 2]); % [x, y, z]
% start = gpuArray([45; 45; 18]); % [x, y, z]
% obstacle_sizes = gpuArray([1, 1, 3, 1, 1]);
% agent_size = 1;
% obstacles = gpuArray([15, 9,  19, 20, 35;...
%                       15, 9,  17, 10, 35;...
%                       3,  10, 8,  5, 16]); % Example obstacle positions
% max_iterations = 100000;
% influence_margins = [2, 2, 2, 2, 2];
% threshold = 2;
% static_saftey_margins = [1, 1, 1, 1, 1];
% circulation_gains = [1, 1, 1, 1, 1];
% smoothing_radius = 1;
% step_size = 0.1;
% 
% % % Call the path planning function
% % path = gpuArray(navigateField3D(start, goal, droneSize, obstacles, obstacleSizes, mapSize));
% 
% % Visualize the map with obstacles
% figure(1);
% ax = gca;
% ax.Projection = "perspective";
% 
% % Plot obstacles with their respective sizes
% for i = 1:size(obstacles, 2)
%     [x, y, z] = sphere(100); % Lower resolution sphere
%     x = gpuArray(x);
%     y = gpuArray(y);
%     z = gpuArray(z);
%     x = x * obstacle_sizes(i) + obstacles(1, i);
%     y = y * obstacle_sizes(i) + obstacles(2, i);
%     z = z * obstacle_sizes(i) + obstacles(3, i);
%     % surf(x, y, z, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeAlpha', 0);
%     surf(x, y, z, 'EdgeAlpha', 0);
%     hold on;
% end
% 
% hold on;
% plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 10);
% plot3(goal(1), goal(2), goal(3), 'bo', 'MarkerSize', 10);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Map with obstacles');
% grid on;
% axis equal;
% 
% % create the path
% path = zeros(3, 1);
% current_position = start;
% 
% % append a point to the path while the agent isn't close enough to the goal yet
% path(:,1) = current_position;
% iteration = 0;
% 
% while norm(current_position - goal) > threshold 
% 
%     normalized_current_ref = navigationField3D(current_position, goal, obstacles, obstacle_sizes, agent_size, influence_margins, static_saftey_margins, circulation_gains, smoothing_radius);
% 
%     % disp("normalized_current_ref");
%     % disp(normalized_current_ref);
% 
%     new_position = current_position + step_size * normalized_current_ref;
%     path(:,end + 1) = new_position;
%     current_position = new_position;
% 
%     % disp(norm(current_position - goal));
% 
%     iteration = iteration + 1;
% 
% end
% 
% disp(size(path));
% z_path_downsample = downsample(path(3,:),100);
% disp("path");
% disp(z_path_downsample);
% % Plot the path
% plot3(path(1,:), path(2,:), path(3,:), 'k-', 'LineWidth', 2);
% legend('Obstacle1','Obstacle2','Obstacle3','Obstacle4','Obstacle5', 'Start', 'Goal', 'Path');
% 
