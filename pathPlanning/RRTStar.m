state_space = MyStateSpace('state_space', dim, [state_min, state_max]);
state_validator = MyStateValidator(state_space);
RRT_planner = plannerRRTStar(state_space, state_validator, GoalBias=goal_bias);
rng(0);
RRT_start_time = tic;
path = plan(RRT_planner, start, goal);




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


if goal_reached
    % find and plot the optimal path in the tree
    path = findOptimalPath(goal, tree_rrt, max_iterations_rrt);
    size_of_path = size(path, 1);    

    % get the time that took the program to run
    rrt_comp_time = toc(rrt_start_time);
    disp(strcat("Path found by RRT* in ", num2str(rrt_comp_time), " seconds:"));
    disp(path);
    
    % plot the tree
    if not(animate_rrt) && rrt_plot
        plotTree(tree_rrt, plot_nodes, tree_color);
    end

    % plot path
    if animate_rrt || rrt_plot
        plot3(path(:,1), path(:,2), path(:,3), "r-", LineWidth=5);
    end
  
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


function [segment_start, segment_direction_normalized, segment_len] = getParametricRepresentationFromTwoPoints(point_1, point_2)
    % get the normalized direction vector, the starting point and the
    % length from the starting and ending point of a line segment
    segment_start = point_1;
    segment_direction = point_2 - point_1;
    segment_len = norm(segment_direction);
    if segment_len ~= 0
        segment_direction_normalized = segment_direction / segment_len;
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
