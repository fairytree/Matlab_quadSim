function [DSM_obstacle, obstacle_lyapunov_threshold] = ...
    obstacleDSM(obstacles, ...
    lyapunov_value, ...
    v, ...
    P, ...
    obstacle_sizes, ...
    agent_size, ...
    kappa_o)

    % Gets the dynamic safety margin of the obstacle constraint, $\Delta^o_i$.
    % also outputs the Lyapunov threshold value

    % Initialize list of Lyapunov threshold values for each obstacle
    lyapunov_thresholds = zeros(size(obstacles, 2), 1);
       
    % Loop over all the obstacles and compute their Lyapunov threshold values
    for i = 1:size(obstacles, 2)
        
        % % original paper linear constraint
        % c = [v(1:3, 1) - obstacles(:, i); [0; 0; 0; 0; 0; 0]];
        % c = c / norm(c);
        % d = c(1:3,1)' * v(1:3, 1) - obstacle_sizes(i) - agent_size - norm(v(1:3, 1) - obstacles(:, i));

        % self-proof linear constraint
        c = -[(v(1:3, 1) - obstacles(:, i)); [0; 0; 0; 0; 0; 0]];
        c = c / norm(c);
        d = c(1:3,1)' * obstacles(:, i) - obstacle_sizes(i) - agent_size;

        lyapunov_thresholds(i) = lyapunovThreshold(v, d, c, P);
    end

    obstacle_lyapunov_threshold = min(lyapunov_thresholds);
    DSM_obstacle = kappa_o * (obstacle_lyapunov_threshold - lyapunov_value);
end

