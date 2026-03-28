% RRT* for 2D unicycle (rectangular obstacles only)
% Expects in workspace: start, goal, rect_obs, buffer
%   rect_obs: Nx4, each row = [x1, y1, x2, y2]
%   buffer:   scalar inflation applied to every obstacle face

rrt_start_time = tic;
dim = 2;


%% RRT parameters

% Search bounds: enclose start, goal, and obstacles with some margin
all_x = [start(1); goal(1); rect_obs(:,1); rect_obs(:,3)];
all_y = [start(2); goal(2); rect_obs(:,2); rect_obs(:,4)];
rrt_margin = 1;  % extra space around the bounding box
bounds_rrt = [min(all_x) - rrt_margin, max(all_x) + rrt_margin;
              min(all_y) - rrt_margin, max(all_y) + rrt_margin];
rrt_search_time = 10;          % seconds
optimize_after = true;
goal_frequency_rrt = 0.5;
max_iterations_rrt = 5000;
step_size_rrt = 0.1;
threshold_rrt = 0.1;
rrt_star_inclusion = true;
max_distance_rrt_star = 1;

% graphical parameters
tree_color = "b";
animate_rrt = true;
plot_nodes = false;
rrt_plot = true;

% internal state
rng(0);
% tree columns: [x, y, cost, parent_idx, dist_to_goal]
tree_rrt = [start', 0, 0, norm(goal - start)];


%% Visualization

if animate_rrt || rrt_plot
    fig = figure('Position', [0 50 900 700]);
    grid on; axis equal; hold on;
    xlim([bounds_rrt(1,1)-0.5, bounds_rrt(1,2)+0.5]);
    ylim([bounds_rrt(2,1)-0.5, bounds_rrt(2,2)+0.5]);
    title('RRT*'); xlabel('x'); ylabel('y');

    % plot start and goal
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2),   'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % plot rectangular obstacles
    for k = 1:size(rect_obs, 1)
        x1 = rect_obs(k,1); y1 = rect_obs(k,2);
        x2 = rect_obs(k,3); y2 = rect_obs(k,4);
        fill([x1 x2 x2 x1], [y1 y1 y2 y2], [0.7 0.7 0.7], ...
             'EdgeColor', 'k', 'LineWidth', 1.5);
    end

    % plot inflated buffer zones
    for k = 1:size(rect_obs, 1)
        bx1 = rect_obs(k,1) - buffer; by1 = rect_obs(k,2) - buffer;
        bx2 = rect_obs(k,3) + buffer; by2 = rect_obs(k,4) + buffer;
        plot([bx1 bx2 bx2 bx1 bx1], [by1 by1 by2 by2 by1], ...
             'r--', 'LineWidth', 1);
    end
end


%% Main loop

goal_reached = false;

for i = 1:max_iterations_rrt
    % random sample (biased towards goal)
    [random_point, ~] = generateRandomPoint(bounds_rrt, goal_frequency_rrt, goal);

    % nearest node in tree
    nearest_idx = getNearestPointIndex(random_point, tree_rrt(:, 1:dim));
    nearest_point = tree_rrt(nearest_idx, 1:dim);

    % steer towards random point
    new_point = getNewPoint(random_point, nearest_point, step_size_rrt);

    % collision check against rectangular obstacles
    [seg_start, seg_dir, seg_len] = getParametricSegment(nearest_point, new_point);
    collided = false;
    for j = 1:size(rect_obs, 1)
        inflated_min = rect_obs(j, 1:dim) - buffer;
        inflated_max = rect_obs(j, dim+1:2*dim) + buffer;
        if lineSegmentAndBoxCollision(seg_start, seg_dir, seg_len, inflated_min, inflated_max)
            collided = true;
            break;
        end
    end

    if ~collided
        nearest_cost = tree_rrt(nearest_idx, dim+1);
        new_cost = nearest_cost + norm(new_point - nearest_point);
        dist_to_goal = norm(goal' - new_point);

        % [x, y, cost, parent, dist_to_goal]
        tree_rrt = [tree_rrt; new_point, new_cost, nearest_idx, dist_to_goal];

        % RRT* rewiring during growth
        if rrt_star_inclusion && ~optimize_after
            tree_rrt = rrtStarRewire(tree_rrt, size(tree_rrt,1), ...
                max_distance_rrt_star, rect_obs, buffer, dim);
        end

        % live animation
        if animate_rrt
            if plot_nodes
                plot(new_point(1), new_point(2), strcat(tree_color, "o"));
            end
            plot([nearest_point(1), new_point(1)], ...
                 [nearest_point(2), new_point(2)], tree_color);
            drawnow;
        end

        if norm(new_point - goal') < threshold_rrt
            goal_reached = true;
        end

        if toc(rrt_start_time) >= rrt_search_time
            break;
        end
    end
end


%% Post-processing

if goal_reached
    % RRT* post-optimization
    if rrt_star_inclusion && optimize_after
        for node_idx = 1:size(tree_rrt, 1)
            tree_rrt = rrtStarRewire(tree_rrt, node_idx, ...
                max_distance_rrt_star, rect_obs, buffer, dim);
        end
    end

    path = findOptimalPath(goal, tree_rrt, max_iterations_rrt, dim);
    size_of_path = size(path, 1);

    rrt_comp_time = toc(rrt_start_time);
    disp(strcat("Path found by RRT* in ", num2str(rrt_comp_time), " seconds"));
    disp(path);

    if ~animate_rrt && rrt_plot
        plotTree2D(tree_rrt, plot_nodes, tree_color, dim);
    end

    if animate_rrt || rrt_plot
        plot(path(:,1), path(:,2), 'r-', 'LineWidth', 4);
        legend('Start', 'Goal', 'Obstacle', 'Buffer', 'Tree', 'Path', 'Location', 'best');
    end

    path = flip(path, 1);  % waypoints from start → goal
else
    disp("No path found within the iteration / time limit.");

    if ~animate_rrt && rrt_plot
        plotTree2D(tree_rrt, plot_nodes, tree_color, dim);
    end

    error('RRT2: path not found');
end


%% ========================  Local Functions  ============================

function idx = getNearestPointIndex(node, tree)
    dists = vecnorm(tree - node, 2, 2);
    [~, idx] = min(dists);
end

function [seg_start, seg_dir, seg_len] = getParametricSegment(p1, p2)
    seg_start = p1;
    d = p2 - p1;
    seg_len = norm(d);
    if seg_len > 0
        seg_dir = d / seg_len;
    else
        seg_dir = d;
    end
end

function [rp, chose_goal] = generateRandomPoint(bounds, goal_freq, goal)
    if rand() < goal_freq
        rp = goal';
        chose_goal = true;
    else
        chose_goal = false;
        rp = zeros(1, size(bounds, 1));
        for k = 1:size(bounds, 1)
            rp(k) = rand() * (bounds(k,2) - bounds(k,1)) + bounds(k,1);
        end
    end
end

function np = getNewPoint(random_pt, nearest_pt, step)
    d = norm(random_pt - nearest_pt);
    if d > step
        np = nearest_pt + step * (random_pt - nearest_pt) / d;
    else
        np = random_pt;
    end
end

function path = findOptimalPath(goal, tree, max_iter, dim)
    idx = getNearestPointIndex(goal', tree(:, 1:dim));
    path = [goal'; tree(idx, 1:dim)];
    for k = 1:max_iter
        idx = tree(idx, dim+2);   % parent column
        if idx == 0, break; end
        path = [path; tree(idx, 1:dim)]; %#ok<AGROW>
    end
end

function collided = lineSegmentAndBoxCollision(seg_start, seg_dir, seg_len, box_min, box_max)
    % Slab-based ray–AABB intersection test (works for any dimension)
    t_enter = -inf;
    t_exit  =  inf;
    ndim = numel(seg_start);
    for k = 1:ndim
        if abs(seg_dir(k)) < 1e-12
            if seg_start(k) < box_min(k) || seg_start(k) > box_max(k)
                collided = false; return;
            end
        else
            t1 = (box_min(k) - seg_start(k)) / seg_dir(k);
            t2 = (box_max(k) - seg_start(k)) / seg_dir(k);
            if t1 > t2, tmp = t1; t1 = t2; t2 = tmp; end
            t_enter = max(t_enter, t1);
            t_exit  = min(t_exit,  t2);
            if t_enter > t_exit
                collided = false; return;
            end
        end
    end
    collided = ~(t_exit < 0 || t_enter > seg_len);
end

function tree = rrtStarRewire(tree, node_idx, max_dist, rect_obs, buffer, dim)
    % Find neighbours within max_dist
    nbrs = [];
    for k = 1:size(tree, 1)
        if k ~= node_idx && norm(tree(k,1:dim) - tree(node_idx,1:dim)) <= max_dist
            nbrs = [nbrs, k]; %#ok<AGROW>
        end
    end

    % Try rewiring each neighbour through node_idx
    for ii = 1:numel(nbrs)
        ni = nbrs(ii);
        candidate_cost = norm(tree(ni,1:dim) - tree(node_idx,1:dim)) + tree(node_idx, dim+1);
        if tree(ni, dim+1) > candidate_cost
            if ~segmentCollidesWithObs(tree(ni,1:dim), tree(node_idx,1:dim), rect_obs, buffer, dim)
                tree(ni, dim+1) = candidate_cost;
                tree(ni, dim+2) = node_idx;
            end
        end
    end

    % Try rewiring node_idx through each neighbour
    for ii = 1:numel(nbrs)
        ni = nbrs(ii);
        candidate_cost = norm(tree(ni,1:dim) - tree(node_idx,1:dim)) + tree(ni, dim+1);
        if tree(node_idx, dim+1) > candidate_cost
            if ~segmentCollidesWithObs(tree(ni,1:dim), tree(node_idx,1:dim), rect_obs, buffer, dim)
                tree(node_idx, dim+1) = candidate_cost;
                tree(node_idx, dim+2) = ni;
            end
        end
    end
end

function hit = segmentCollidesWithObs(p1, p2, rect_obs, buffer, dim)
    [s, d, l] = getParametricSegment(p1, p2);
    hit = false;
    for j = 1:size(rect_obs, 1)
        inflated_min = rect_obs(j, 1:dim) - buffer;
        inflated_max = rect_obs(j, dim+1:2*dim) + buffer;
        if lineSegmentAndBoxCollision(s, d, l, inflated_min, inflated_max)
            hit = true; return;
        end
    end
end

function plotTree2D(tree, plot_nodes, color, dim)
    for k = 1:size(tree, 1)
        if plot_nodes
            plot(tree(k,1), tree(k,2), strcat(color, "o"));
        end
        pidx = tree(k, dim+2);
        if pidx ~= 0
            plot([tree(pidx,1), tree(k,1)], ...
                 [tree(pidx,2), tree(k,2)], color);
        end
    end
end
