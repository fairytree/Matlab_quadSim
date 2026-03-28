%% RRT parameters
% algorithm parameters
bounds_rrt = [-1, 4;
              -1, 4;
              -1, 2.5]; % boundaries RRT can search in (each row is a upper & lower bound pair)
% bounds_rrt = [-1, 5;
%               -1, 5;
%               -1, 3];
rrt_search_time = 10; % in seconds
optimize_after = true; % if true, the tree is optimized after done generating.
% Otherwise, it is optimized at every iteration during the generation
goal_frequency_rrt = 0.5;
max_iterations_rrt = 2000;
step_size_rrt = 0.1;
threshold_rrt = 0.1;
rrt_star_inclusion = true; % use RRT*, instead of regular RRT
max_distance_rrt_star = 1; % the distance within whcih RRT* will try to find the optimal rewiring
% graphical parameters
tree_color = "b";
animate_rrt = false; % plot the tree while running
plot_nodes = false; % plot the nodes as circles
rrt_plot = false; % plot the tree when done

%% potential field parameters
max_iterations_potential_field = 10000;
potential_field_step_size = 0.05;
smoothing_radius = 1;
influence_margins =  0.2 * ones(size(obstacle_sizes)); 
static_safety_margins = 0.5 * influence_margins; % static_saftey_margins has to be smaller than influence_margins
circulation_gains =  0.3 * static_safety_margins;

path_planners = [1]; % 1: RRT*; 2: potential field
safety_margin = 0.02;
safety_margins_RRT = static_safety_margins + agent_size;