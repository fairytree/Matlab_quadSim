safety_margin_universal = 0.02;
safety_margin_universal_multiple = [0.1, 0.03, 0.005]; % [0.1, 0.03, 0.005] Comment out if not for compare multiple paths
static_safety_margins = safety_margin_universal * ones(size(obstacle_sizes));
safety_margins_RRT = static_safety_margins + agent_size;

% valid strings: "RRT*", "Pot. Field"
path_planners = ["RRT*"];
% path_planners = ["Pot. Field"];