% format for the list of rectangular prisms:
% each prism is represented by a pair of vertices--the one with the lowest x, y, z coordinates and the one with the highest
% call these points (x1, y1, z1) and (x2, y2, z2) respectively
% the list looks like this:
% [
%     xa1, ya1, za1, xa2, ya2, za2; % obstacle A
%     xb1, yb1, zb1, xb2, yb2, zb2; % obstacle B
%     ...
% ]
% in the case of a unicycle, it's 2D, so there's no z-coordinate
rectangular_obs = [2, 0, 4, 4; 0, 1, 1, 2];
buffer = 0.3;
N = [5]; % prediction horizon(s)
dim = 2; % dimension, either 2 or 3
n_x = 2; % number of states
n_u = 2; % number of control inputs
k = 1; % scaling factor for the terminal controller
q = 1; % scaling factor for stage cost
max_speed = 0.5;
max_iters = 5000;
params = struct(...
    'rectangular_obs', rectangular_obs,...
    'buffer', buffer,...
    'N', N,...
    'dim', dim,... 
    'n_x', n_x,...
    'n_u', n_u,...
    'k', k,...
    'q', q,...
    'max_speed', max_speed,...
    'max_iters', max_iters...
);
params.deci_var_length = N * (n_x + n_u) + n_x;
params.lyapunov_thresh = max_speed / k; % TODO implement threshold for obstacle constraints
params.quad_cost_mat = getQuadCostMat(params);
[params.lower_bound, params.upper_bound] = getBounds(params);

decision_variables_init = repmat([x_init; u_init], prediction_horizon_MPC, 1);
decision_variables_init = [decision_variables_init; x_init];
auxi_ref_list = linspace(1, size(path, 1), prediction_horizon_MPC + 1);
if prediction_horizon_MPC > PathFG_max_N
    for k = 1:prediction_horizon_MPC
        auxi_ref = auxi_ref_list(k);
        p = getVPosition(auxi_ref, path);
        idx = (k-1)*13 + 1;
        decision_variables_init(idx:idx+2) = p(1:3);
    end
end

Q_MPC_diagonal = 5 * [2, 2, 2, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5]; % 5 * [2, 2, 2, 1, 1, 1, 1, 1, 1];
Q_MPC = diag(Q_MPC_diagonal);

R_MPC_diagonal = 0.1 * ones(1, 4);
R_MPC = diag(R_MPC_diagonal);

Q_LQR = Q_MPC;
R_LQR = R_MPC;

function [lower_bound, upper_bound] = getBounds(params)
    N = params.N;
    n_x = params.n_x;
    max_speed = params.max_speed;
    state_lower_bound = -Inf * ones(n_x,1);
    input_lower_bound = [-2*pi; 0]; % 720-degree rotation space
    lower_bound = [repmat([state_lower_bound;input_lower_bound],N,1); state_lower_bound];
    state_upper_bound = Inf * ones(n_x,1);
    input_upper_bound = [2*pi; max_speed];
    upper_bound = [repmat([state_upper_bound;input_upper_bound],N,1); state_upper_bound];
end

% the matrix in the quadractic term of the cost function
function quad_cost_mat = getQuadCostMat(params)
    n_x = params.n_x;
    n_u = params.n_u;
    N = params.N;
    q = params.q;
    stage_diag = [q*ones(n_x,1); zeros(n_u,1)];
    quad_cost_mat_diag = [repmat(stage_diag,N,1); ones(n_x,1)];
    quad_cost_mat = diag(quad_cost_mat_diag);
end