
%% Potential Field
tic


%% Parameters
max_iterations_potential_field = 10000;
potential_field_step_size = 0.05;
smoothing_radius = 1;
influence_margins =  0.2 * ones(size(obstacle_sizes)); 
static_safety_margins = 0.5 * influence_margins; % static_saftey_margins has to be smaller than influence_margins
circulation_gains =  0.3 * static_safety_margins;


%% Main

current_position = start;
iter = 0;
path = [];
while (norm(current_position - goal) > goal_threshold) && iter < max_iterations_potential_field

    new_position = current_position + potential_field_step_size * ...
        getNextRef(current_position, ...
        goal, ...
        goal_threshold,...
        obstacles, ...
        obstacle_sizes, ...
        agent_size, ...
        influence_margins,...
        static_safety_margins, ...
        circulation_gains,...
        smoothing_radius); 

    path(end + 1, :) = new_position';
    current_position = new_position;

    iter = iter + 1;

end


size_of_path = size(path, 1);

toc

disp("path");
disp(path);


%% Functions

function  normalized_current_ref = getNextRef( ...
    current_ref, ...
    goal, ...
    goal_threshold,...
    obstacles, ...
    obstacle_sizes, ...
    agent_size, ...
    influence_margins,...
    static_safety_margins, ...
    circulation_gains,...
    smoothing_radius)

    current_ref_x_y_z = current_ref(1:3);

    % check if goal reached
    if(norm(current_ref_x_y_z - goal)) < goal_threshold
        normalized_current_ref = [0; 0; 0];
        return;
    end
 
    repulsion_field_obstacles_nonconservative = [0; 0; 0];
    repulsion_field_obstacles_conservative = [0; 0; 0];

    % add up each repulsion field for every indiviual obstacle
    number_of_obstacles = numel(obstacles(1,:));
    for j = 1:number_of_obstacles
        
        obstacle_position = obstacles(:,j);
        obstacle_size = obstacle_sizes(j);
        distance_from_obstacle = distanceFromObstacle(obstacle_position, current_ref_x_y_z,...
            obstacle_size, agent_size);

        % calculate conservative repulsion field for obstacles
        repulsion_field_obstacles_conservative = repulsion_field_obstacles_conservative +...
            max((influence_margins(j) - distance_from_obstacle) /...
            (influence_margins(j) - static_safety_margins(j)), 0)...
            * normalize_(current_ref_x_y_z - obstacles(:,j), 0);


        % calculate nonconservative repulsion field for obstacles
        if influence_margins(j) >= distance_from_obstacle
            repulsion_field_obstacles_nonconservative = repulsion_field_obstacles_nonconservative + circulation_gains(j) * normalize_(...
            [-obstacle_position(2) + current_ref_x_y_z(2) + obstacle_position(3) - current_ref_x_y_z(3);...
            obstacle_position(1) - current_ref_x_y_z(1) - obstacle_position(3) + current_ref_x_y_z(3);...
            -obstacle_position(1) + current_ref_x_y_z(1) + obstacle_position(2) - current_ref_x_y_z(2)], 0);
        end

    end

    % calculate total repulsion field for obstacles
    repulsion_field_obstacles = repulsion_field_obstacles_conservative + ...
        repulsion_field_obstacles_nonconservative;
    
    % calculate attraction and repulsion fields
    attraction_field = normalize_(goal - current_ref_x_y_z, smoothing_radius);

    % return the total navigation field
    normalized_current_ref = attraction_field + repulsion_field_obstacles;
    
end


function normalized_vector = normalize_(vector, smoothing_radius)

    normalized_vector = vector / max(norm(vector), smoothing_radius);

end


function distance_from_obstacle = distanceFromObstacle(obstacle_position, agent_position, obstacle_size, agent_size)

    distance_from_obstacle = norm(agent_position - obstacle_position) - agent_size - obstacle_size;

end

