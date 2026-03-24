% RRT (stop RRT* when time out)

rrt_start_time = tic;


%% RRT parameters

% algorithm parameters
bounds_rrt = [-1, 5;
              -1, 5;
              -1, 3]; % boundaries RRT can search in (each row is a upper & lower bound pair)
rrt_search_time = 10; % in seconds
optimize_after = true; % if true, the tree is optimized after done generating.
% Otherwise, it is optimized at every iteration during the generation
goal_frequency_rrt = 0.5;
max_iterations_rrt = 1000;
step_size_rrt = 0.1;
threshold_rrt = 0.1;
rrt_star_inclusion = true; % use RRT*, instead of regular RRT
max_distance_rrt_star = 1; % the distance within whcih RRT* will try to find the optimal rewiring

% graphical parameters
tree_color = "b";
animate_rrt = false; % plot the tree while running
plot_nodes = false; % plot the nodes as circles
rrt_plot = false; % plot the tree when done

% internal parameters
rng(0); % seed of the random number generator for reproducibility
tree_rrt = [start', 0, 0, norm(goal - start)];


%% Visualization

if animate_rrt || rrt_plot
    % create figure for visualization
    fig = figure('Position', [0 50 1200 800]);
    hg = gca;
    hg.Projection = "perspective";
    view(68,53);
    grid on;
    axis equal;
    hold on;
    xlim([start(1)-1 goal(1)+1]);
    ylim([start(2)-1 goal(2)+1]);
    zlim([start(3)-1 goal(3)+1.2]);
    
    title('RRT')
    xlabel('x');
    ylabel('y');
    zlabel('z');
    
    % plot start and goal points
    plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 10);
    plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize', 10);
    
    % plot obstacles with their respective sizes
    for i = 1:size(obstacles, 2)
        [x, y, z] = sphere(25);
        x = x * obstacle_sizes(i) + obstacles(1, i);
        y = y * obstacle_sizes(i) + obstacles(2, i);
        z = z * obstacle_sizes(i) + obstacles(3, i);
    
        surf(x, y, z, 'EdgeAlpha', 0);
    end
end


%% Main

goal_reached = false;

for i = 1:max_iterations_rrt
    % generate random point
    [random_point, ~] = generateRandomPoint(bounds_rrt, goal_frequency_rrt, goal);

    % Find the nearest point in tree_rrt to the random point
    nearest_idx = getNearestPointIndex(random_point, tree_rrt(:,1:3));
    nearest_point = tree_rrt(nearest_idx,1:3);

    % Move towards the random point by at most step_size_rrt
    new_point = getNewPoint(random_point, nearest_point, step_size_rrt);

    % check if whether or not the line segment has collided with an
    % obstacle
    [segment_start, segment_direction_normalized, segment_length] = getParametricRepresentationFromTwoPoints(nearest_point, new_point);
    collided = false;
    for j = 1:size(obstacles, 2)
        if lineSegmentAndSphereCollision(segment_start', ...
                segment_direction_normalized', segment_length, ...
                obstacle_sizes(j) + safety_margins_RRT(j), obstacles(:,j))
            collided = true;
        end
    end

    % add node to tree if it's not colliding with anything
    if not(collided)
        % the cost of the nearest point
        nearest_point_cost = tree_rrt(nearest_idx,4);

        % calculate the cost of the new point
        new_point_cost = nearest_point_cost + norm(new_point - nearest_point);

        % calculate its distance from the goal
        distance_from_goal = norm(goal - new_point);

        % Add the new point to tree_rrt with its cost, parent node and
        % distance from the goal
        tree_rrt = [tree_rrt; new_point, new_point_cost, nearest_idx, distance_from_goal];

        % optimize the tree during generation using RRT*
        if rrt_star_inclusion && not(optimize_after)
            tree_rrt = RRTStar(tree_rrt, size(tree_rrt, 1),...
                max_distance_rrt_star, obstacles, obstacle_sizes, safety_margins_RRT);
        end

        % Plot the new point and edge
        if animate_rrt
            if plot_nodes
                plot3(new_point(1), new_point(2), new_point(3), strcat(tree_color, "o"));
            end

            new_edge = plot3([nearest_point(1), new_point(1)], ...
                [nearest_point(2), new_point(2)], ...
                [nearest_point(3), new_point(3)], tree_color);

            drawnow;
        end        

        % check if the new point is close to the goal
        if norm(new_point - goal') < threshold_rrt
            goal_reached = true;
        end
        
        % check if the time is up
        if toc(rrt_start_time) >= rrt_search_time
            break
        end
    end
end

if goal_reached
    % optimize the tree using RRT*
    if rrt_star_inclusion && optimize_after
        for node_idx = 1:size(tree_rrt, 1)
            tree_rrt = RRTStar(tree_rrt, node_idx,...
                max_distance_rrt_star, obstacles, obstacle_sizes, safety_margins_RRT);
        end
    end

    % find and plot the optimal path in the tree
    optimal_path = findOptimalPath(goal, tree_rrt, max_iterations_rrt);
    optimal_path = optimal_path(2:end, :); % !!! The end point is duplicate. 
    size_of_optimal_path = size(optimal_path, 1);

    % get the time that took the program to run
    rrt_comp_time = toc(rrt_start_time);

    disp(strcat("Path found by RRT* in ", num2str(rrt_comp_time), " seconds:"));
    disp(optimal_path);
    
    % plot the tree
    if not(animate_rrt) && rrt_plot
        plotTree(tree_rrt, plot_nodes, tree_color);
    end

    % plot path
    if animate_rrt || rrt_plot
        plot3(optimal_path(:,1), optimal_path(:,2), optimal_path(:,3), "r-", LineWidth=5);
    end
  
    optimal_path_pathFG = flip(optimal_path, 1); % store waypoints from start to goal
else
    disp("A path wasn't found within the maximum number of iterations or the maximum allocated time");

    % plot the tree
    if not(animate_rrt) && rrt_plot
        plotTree(tree_rrt, plot_nodes, tree_color);
    end

    assert(1 + 1 == 3);
end


%% Functions

function index = getNearestPointIndex(node, tree)
    % Find the index of the nearest point in tree_rrt to the given point    
    distances = vecnorm(tree - node, 2, 2);
    [~, index] = min(distances);
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

function [random_point, chosen_goal] = generateRandomPoint(bounds_rrt, goal_frequency, goal)
    if rand() < goal_frequency
        % sometimes sample the goal
        random_point = goal';
        chosen_goal = true;
    else
        % Generate a random point
        random_point = zeros(1, size(bounds_rrt, 1));
        chosen_goal = false;

        for i = 1:size(bounds_rrt, 1)
            random_point(1,i) = rand() * (bounds_rrt(i,2) - bounds_rrt(i,1)) + bounds_rrt(i,1);
        end
    end

    % disp(random_point);
end

function path = findOptimalPath(goal, tree, max_iterations)
    % find the optimal path
    current_point_index = getNearestPointIndex(goal', tree(:,1:3));
    current_point = tree(current_point_index,1:3);
    path = [goal'; current_point];

    for i = 1:max_iterations
        % connect to its parent node
        current_point_index = tree(current_point_index,5);

        % break if we reach the start
        if current_point_index == 0
            break
        end

        current_point = tree(current_point_index,1:3);
        path = [path; current_point];
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
    obstacles, obstacle_sizes, safety_margins_RRT)
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
                        obstacle_sizes(j) + safety_margins_RRT(j), obstacles(:,j)')
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
                        obstacle_sizes(j) + safety_margins_RRT(j), obstacles(:,j)')
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

function new_tree = addNodeToTree(tree, node_position, cost, parent_index, distance_from_goal)
    % appends a new node to tree
    % this will append the new node depending on distance_from_goal
    
    % find where to insert the node
    insertion_index = gerInsertionIndex(tree(:,6), distance_from_goal);

    % combine the information together into a row vector
    node = [node_position, cost, parent_index, distance_from_goal];

    % add the node to the tree
    new_tree = insertNode(tree, node, insertion_index);
end

function insertion_index = gerInsertionIndex(array, number)
    % finds where to insert number into array
    % array is assumed to be already sorted and nonempty
    % this function uses divide and conquer

    i = 1;
    number_of_elements = numel(array);
    j = number_of_elements;

    while i ~= j
        m = floor((i + j)/2);

        if array(m) == number
            i = m;
            break;
        elseif number > array(m)
            i = m + 1;
        else
            j = m;
        end
    end

    if number > array(number_of_elements)
        insertion_index = number_of_elements + 1;
    end

    insertion_index = i;
end

function new_tree = insertNode(tree, node, insertion_index)
    % inserts a node into a tree
    if insertion_index ~= 1 && insertion_index <= size(tree, 1)
        new_tree = [tree(1:insertion_index - 1,:); node; tree(insertion_index:end,:)];
    elseif insertion_index == 1
        new_tree = [node; tree];
    else
        new_tree = [tree; node];
    end
end

%% Tests

% both should output 1
% disp(lineSegmentAndSphereCollision([-4.98036;4.49512;4], ...
%     [6.7484;-7.6048;1.2486]/norm([6.7484;-7.6048;1.2486]), ...
%     norm([6.7484;-7.6048;1.2486]),5,[-5;-1;5]));
% disp(lineSegmentAndSphereCollision([-2,0,0],[1,0,0],4,1,[0,0,0]));
