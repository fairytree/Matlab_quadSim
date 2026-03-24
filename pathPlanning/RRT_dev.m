clear;
close all;
clc;


%% Variables

% NOTE: start and goal must be row vectors
% Define the start and goal points (3D)
start = [0, 0, 0];
goal = [10, 10, 10];

% color of the tree
tree_color = "blue";

% define how often the goal is chosen as the random point
goal_frequency_rrt = 0.5;

% maximum distance for which RRT* would consider as a neighbor
max_distance_rrt_star = 1;

% whether or not to optimize tree_rrt using RRT*
rrt_star_inclusion = 1;

% whether or not to animate the tree expanding
animate_rrt = 0;

% Define the maximum number of iterations
max_iterations_rrt = 100000;

% Define the step size
step_size_rrt = 0.1;

% maximum distance from the goal for which a point can be considered near
% the goal
threshold_rrt = 1;

% Initialize the tree with the start point (the fourth value represents
% how away it is from start, and is used to find the optimal path later,
% and the fifth value represents the index of its parent node)
tree_rrt = [start, 0, 0];

% obstacle sizes and positions
obstacles = 5*[1.5, 0.9,  8/5, 2.0, 2.5;...
             1.5, 0.9,  8/5, 1.0, 2.5;...
             0.3,  1, 8/5,  0.5, 2.0];
agent_size = 0.08;
safety_margins = [0.1, 0.1, 0.1, 0.1, 0.1] + agent_size;
obstacle_sizes = 7*[0.1, 0.1, 0.3, 0.1, 0.1];

% seed of the random number generator for reproducibility
rng(0);

% whether or not to plot the nodes
plot_nodes = false;


%% Visualization

% Create figure for visualization
fig = figure('Position', [0 50 1200 800]);
hg = gca;
hg.Projection = "perspective";
view(68,53);
grid on;
axis equal;
hold on;
xlim([start(1)-1 goal(1)+1]);
ylim([start(2)-1 goal(2)+1]);
zlim([start(3)-1 goal(3)+1]);

title('RRT')
xlabel('x');
ylabel('y');
zlabel('z');

% Plot start and goal points
plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 10);
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize', 10);

% Plot obstacles with their respective sizes
    for i = 1:size(obstacles, 2)
        [x, y, z] = sphere(25);
        x = x * obstacle_sizes(i) + obstacles(1, i);
        y = y * obstacle_sizes(i) + obstacles(2, i);
        z = z * obstacle_sizes(i) + obstacles(3, i);

        surf(x, y, z, 'EdgeAlpha', 0);
    end

%% Main

% hold off;
goal_reached = false;

for i = 1:max_iterations_rrt
    % boundaries for the generated random points
    bounds = [0, 12; 0, 12; 0, 15];

    % generate random point
    random_point = generateRandomPoint(bounds, goal_frequency_rrt, goal);

    % Find the nearest point in tree_rrt to the random point
    nearest_index = getNearestPointIndex(random_point, tree_rrt(:,1:3));
    nearest_point = tree_rrt(nearest_index,1:3);

    % Move towards the random point by at most step_size_rrt
    new_point = getNewPoint(random_point, nearest_point, step_size_rrt);

    % check if whether or not the line segment has collided with an
    % obstacle
    [segment_start, segment_direction_normalized, segment_length] = getParametricRepresentationFromTwoPoints(nearest_point, new_point);
    collided = false;
    for j = 1:size(obstacles, 2)
        if lineSegmentAndSphereCollision(segment_start', ...
                segment_direction_normalized', segment_length, ...
                obstacle_sizes(j) + safety_margins(j), obstacles(:,j))
            collided = true;
        end
    end

    % add node to tree if it's not colliding with anything
    if not(collided)
        % the cost of the nearest point
        nearest_point_cost = tree_rrt(nearest_index,4);

        % calculate the cost of the new point
        new_point_cost = nearest_point_cost + norm(new_point - nearest_point);

        % Add the new point to tree_rrt with its cost and parent node
        tree_rrt = [tree_rrt; new_point, new_point_cost, nearest_index];

        % optimize the tree using RRT*
        if rrt_star_inclusion
            tree_rrt = RRTStar(tree_rrt, size(tree_rrt, 1), ...
                max_distance_rrt_star, obstacles, obstacle_sizes, safety_margins);
        end

        % Plot the new point and edge
        if animate_rrt
            if plot_nodes
                plot3(new_point(1), new_point(2), new_point(3), 'bo');
            end

            plot3([nearest_point(1), new_point(1)], [nearest_point(2), new_point(2)], [nearest_point(3), new_point(3)], 'b-');
            drawnow;
        end        

        % Check if the new point is close to the goal
        if norm(new_point - goal) < threshold_rrt
            goal_reached = true;
            break;
        end
    end
end

if goal_reached
    disp("Goal reached!");

    % plot the tree
    if not(animate_rrt)
        plotTree(tree_rrt, plot_nodes, tree_color);
    end

    % find and plot the optimal path in the tree
    optimal_path = findOptimalPath(start, goal, tree_rrt, max_iterations_rrt);
    plot3(optimal_path(:,1), optimal_path(:,2), optimal_path(:,3), "r-", "LineWidth", 5);
else
    disp("Goal not reached");
end


%% Functions

function index = getNearestPointIndex(point, tree_rrt)
    % Find the index of the nearest point in tree_rrt to the given point
    distances = vecnorm(tree_rrt(:,1:3) - point, 2, 2);
    [~, index] = min(distances);
end

function collided = lineSegmentAndSphereCollision(segment_start, ...
    segment_direction_normalized, segment_length, sphere_radius, ...
    sphere_center)
    % check if the given line segment and sphere is colliding

    % see https://education.siggraph.org/static/HyperGraph/raytrace/rtinter1.htm
    b = 2 * (segment_direction_normalized(1) * (segment_start(1) - sphere_center(1)) ...
        + segment_direction_normalized(2) * (segment_start(2) - sphere_center(2)) ...
        + segment_direction_normalized(3) * (segment_start(3) - sphere_center(3)));
    c = (segment_start(1) - sphere_center(1))^2 + ...
        (segment_start(2) - sphere_center(2))^2 + ...
        (segment_start(3) - sphere_center(3))^2 - sphere_radius^2;

    discriminant = b^2 - 4 * c;

    % see if discriminant is negative or not
    if discriminant < 0
        collided = false;
    else
        t_0 = (-b + sqrt(discriminant)) / 2;
        t_1 = (-b - sqrt(discriminant)) / 2;

        % see if the line collided with the sphere on the line segment
        if (0 <= t_0 && t_0 <= segment_length) || (0 <= t_1 && t_1 <= segment_length)
            collided = true;
        else
            collided = false;
        end
    end
end

function [segment_start, segment_direction_normalized, segment_length] = getParametricRepresentationFromTwoPoints(point_1, point_2)
    % get the normalized direction vector, the starting point and the
    % length from the starting and ending point of a line segment
    segment_start = point_1;
    segment_direction = point_2 - point_1;
    segment_length = norm(segment_direction);
    if segment_length ~= 0
        segment_direction_normalized = segment_direction / segment_length;
    else
        segment_direction_normalized = segment_direction;
    end
end

function random_point = generateRandomPoint(bounds, goal_frequency, goal)
    if rand() < goal_frequency
        % sometimes sample the goal
        random_point = goal;
    else
        % Generate a random point
        random_point = [];

        for i = 1:size(bounds, 1)
            random_point = [random_point, rand() * (bounds(i,2) - bounds(i,1)) + bounds(i,1)];
        end
    end
end

function optimal_path = findOptimalPath(start, goal, tree, max_iterations)
    % find the optimal path
    current_point_index = getNearestPointIndex(goal, tree);
    current_point = tree(current_point_index,1:3);
    optimal_path = [goal; current_point];

    for i = 1:max_iterations
        % connect to its parent node
        current_point_index = tree(current_point_index,5);
        current_point = tree(current_point_index,1:3);
        optimal_path = [optimal_path; current_point];

        % break if we reach the start
        if current_point == start
            break
        end
    end
end

function new_point = getNewPoint(random_point, nearest_point, step_size)
    distance = norm(random_point - nearest_point);
    if distance > step_size
        direction = (random_point - nearest_point) / distance;
        new_point = nearest_point + step_size * direction;
    else
        new_point = random_point;
    end
end

function neighbor_indices = getNeighboringNodeIndices(tree, node_index, max_distance)
    % get the indices of the neighboring nodes of the node specified by
    % by node_index
    neighbor_indices = [];

    for k = 1:size(tree, 1)
        if ~(k == node_index) && (norm(tree(k,1:3) - tree(node_index,1:3)) <= max_distance)
            neighbor_indices = [neighbor_indices, k];
        end
    end
end

function optimized_tree = RRTStar(tree, node_index, max_distance, ...
    obstacles, obstacle_sizes, safety_margins)
    % optimizes tree using the RRT* algorithm
    neighbor_indices = getNeighboringNodeIndices(tree, node_index, max_distance);

    % decide whether or not to rewire neighbors to the new node
    for i = 1:numel(neighbor_indices)
        new_cost = norm(tree(neighbor_indices(i),1:3) ...
                - tree(node_index,1:3)) + tree(node_index,4);

        if tree(neighbor_indices(i),4) > new_cost
            % check if the new connection is valid
            [segment_start, segment_direction_normalized, segment_length] = ...
                getParametricRepresentationFromTwoPoints( ...
                tree(neighbor_indices(i),1:3), tree(node_index,1:3));
            collided = false;

            for j = 1:size(obstacles, 2)
                if lineSegmentAndSphereCollision(segment_start, ...
                        segment_direction_normalized, segment_length, ...
                        obstacle_sizes(j) + safety_margins(j), obstacles(:,j)')
                    collided = true;
                end
            end

            if not(collided)
                % change the parent node and cost of the neighboring index
                tree(neighbor_indices(i),4) = new_cost;
                tree(neighbor_indices(i),5) = node_index;
            end
        end
    end

    % decide which neighbor the new node should be rewired to (if any of
    % them has a lower cost if wired to them than to the nearest node)
    for i = 1:numel(neighbor_indices)
        new_cost = norm(tree(neighbor_indices(i),1:3) ...
                - tree(node_index,1:3)) + tree(neighbor_indices(i),4);

        if tree(node_index,4) > new_cost
            % check if the new connection is valid
            [segment_start, segment_direction_normalized, segment_length] = ...
                getParametricRepresentationFromTwoPoints( ...
                tree(neighbor_indices(i),1:3), tree(node_index,1:3));
            collided = false;

            for j = 1:size(obstacles, 2)
                % disp(segment_start);
                % disp(segment_direction_normalized);
                % disp(segment_length);
                % disp(obstacle_sizes(j));
                % disp(obstacles(:,j)');

                if lineSegmentAndSphereCollision(segment_start, ...
                        segment_direction_normalized, segment_length, ...
                        obstacle_sizes(j) + safety_margins(j), obstacles(:,j)')
                    collided = true;
                end
            end

            if not(collided)
                % change the parent node and cost of the neighboring index
                tree(node_index,4) = new_cost;
                tree(node_index,5) = neighbor_indices(i);
            end
        end
    end

    optimized_tree = tree;
end

function plotTree(tree, plot_nodes, color)
    % plots tree
    for i = 1:size(tree, 1)
        if plot_nodes
            plot3(tree(i,1), tree(i,2), tree(i,3), "bo");
        end
        
        if tree(i,5) ~= 0
            plot3([tree(tree(i,5),1), tree(i,1)], ...
                [tree(tree(i,5),2), tree(i,2)], ...
                [tree(tree(i,5),3), tree(i,3)], color);
        end
    end
end


%% Tests

% both should output 1
% disp(lineSegmentAndSphereCollision([-4.98036;4.49512;4], ...
%     [6.7484;-7.6048;1.2486]/norm([6.7484;-7.6048;1.2486]), ...
%     norm([6.7484;-7.6048;1.2486]),5,[-5;-1;5]));
% disp(lineSegmentAndSphereCollision([-2,0,0],[1,0,0],4,1,[0,0,0]));
