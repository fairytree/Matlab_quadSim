function trajectoryBasedDSM_value = trajectoryBasedDSM( ...
    predicted_traj,...
    U,...
    agent_size, ...
    obstacle_position, ...
    obstacle_sizes, ...
    safety_margin, ...
    x_min, ...
    x_max, ...
    u_min,...
    u_max,...
    delta_FG,...
    scaling_factor_traj_DSM_obstacle, ...
    scaling_factor_traj_DSM_x_bounds, ...
    scaling_factor_traj_DSM_U_bounds)
    
    trajectoryBasedDSM_value = realmax("double");

    % Dist from obstacles
    for i = 1:size(predicted_traj, 2)
        
        for j = 1:size(obstacle_position, 2)

            % dist_ob = norm(predicted_traj(1:3, i) - obstacle_position(:,j)) ...
            % - agent_size - obstacle_sizes(j) - safety_margin;

            dist_ob = norm(predicted_traj(1:3, i) - obstacle_position(:,j)) ...
            - agent_size - obstacle_sizes(j);
            
            trajectoryBasedDSM_value = min(trajectoryBasedDSM_value, scaling_factor_traj_DSM_obstacle * dist_ob);
        end                     
    end
    % disp("obstacles");
    % disp(trajectoryBasedDSM_value);   

    % Dist from X boundary (normalize to 0-1 and then scale)
    x_min_strict = (1 - delta_FG) * x_min;
    x_max_strict = (1 - delta_FG) * x_max;
    x_bounds_range = x_max_strict - x_min_strict;
    for i = 1:size(predicted_traj, 2)
        dist_x_lower_bounds = (predicted_traj(:, i) - x_min_strict) ./ x_bounds_range;
        dist_x_upper_bounds = (x_max_strict - predicted_traj(:, i)) ./ x_bounds_range;
        sdist_x_bounds = min(min(dist_x_lower_bounds), min(dist_x_upper_bounds));
        trajectoryBasedDSM_value = min(trajectoryBasedDSM_value, scaling_factor_traj_DSM_x_bounds * sdist_x_bounds);
    end
    % disp("sdist_x_bounds");
    % disp(trajectoryBasedDSM_value);  
    

    % Dist from U boundary (normalize to 0-1 and then scale)
    u_min_strict = (1 - delta_FG) * u_min;
    u_max_strict = (1 - delta_FG) * u_max;
    u_bounds_range = u_max_strict - u_min_strict;
    for i = 1:size(U, 2)        
        dist_U_lower_bounds = (U(:, i) - u_min_strict) ./ u_bounds_range;
        dist_U_upper_bounds = (u_max_strict - U(:, i)) ./ u_bounds_range;
        sdist_U_bounds = min(min(dist_U_lower_bounds), min(dist_U_upper_bounds));
        trajectoryBasedDSM_value = min(trajectoryBasedDSM_value, scaling_factor_traj_DSM_U_bounds * sdist_U_bounds);
    end
    % disp("sdist_U_bounds");
    % disp(trajectoryBasedDSM_value);  
    
end