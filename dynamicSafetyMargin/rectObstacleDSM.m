function [DSM_rect_obstacle, rect_obstacle_lyapunov_threshold] = ...
    rectObstacleDSM(rect_obs, lyapunov_value, v, P, agent_size, kappa_o)
% Gets the dynamic safety margin for rectangular prism obstacles.
% Uses the same linearization approach as obstacleDSM but for
% axis-aligned boxes.
%
% Inputs:
%   rect_obs            - N_r x 6 matrix, each row: [x_min y_min z_min x_max y_max z_max]
%   lyapunov_value      - scalar, current Lyapunov value
%   v                   - 9x1 reference state (positions in v(1:3))
%   P                   - 9x9 Lyapunov matrix
%   agent_size          - scalar, agent radius
%   kappa_o             - scalar, DSM gain
%
% Outputs:
%   DSM_rect_obstacle               - dynamic safety margin
%   rect_obstacle_lyapunov_threshold - minimum Lyapunov threshold across all rect obstacles

N_r = size(rect_obs, 1);

if N_r == 0
    rect_obstacle_lyapunov_threshold = Inf;
    DSM_rect_obstacle = Inf;
    return;
end

lyapunov_thresholds = zeros(N_r, 1);
ref_pos = v(1:3, 1);

for i = 1:N_r
    box_min = rect_obs(i, 1:3)';
    box_max = rect_obs(i, 4:6)';

    % Inflate the box by agent_size (same as sphere: obs_size + agent_size)
    inflated_min = box_min - agent_size;
    inflated_max = box_max + agent_size;

    % Find closest point on the inflated box to the reference position
    closest_pt = max(inflated_min, min(ref_pos, inflated_max));

    % Direction from closest point to reference position
    diff = ref_pos - closest_pt;
    dist = norm(diff);

    if dist > 1e-8
        % Reference is outside the inflated box
        n_vec = diff / dist;
    else
        % Reference is inside the inflated box — push out along
        % the axis with the smallest penetration depth
        penetration = min(ref_pos - inflated_min, inflated_max - ref_pos);
        [~, min_axis] = min(penetration);
        n_vec = zeros(3, 1);
        if ref_pos(min_axis) - inflated_min(min_axis) < inflated_max(min_axis) - ref_pos(min_axis)
            n_vec(min_axis) = -1;
        else
            n_vec(min_axis) = 1;
        end
    end

    % Linearized constraint: c' * x <= d
    % Same convention as obstacleDSM: c = -[n_vec; 0;0;0;0;0;0] / norm(...)
    c = -[n_vec; zeros(6, 1)];
    c = c / norm(c);
    d = c(1:3, 1)' * closest_pt;

    lyapunov_thresholds(i) = lyapunovThreshold(v, d, c, P);
end

rect_obstacle_lyapunov_threshold = min(lyapunov_thresholds);
DSM_rect_obstacle = kappa_o * (rect_obstacle_lyapunov_threshold - lyapunov_value);
end
