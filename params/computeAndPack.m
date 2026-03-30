% compute parameters and pack them into a single structure
params = struct(...
    'rect_obs', rect_obs,...
    'buffer', buffer,...
    'N', N,...
    'dim', dim,... 
    'n_x', n_x,...
    'n_u', n_u,...
    'k', k,...
    'q', q,...
    'r_weight', r_weight,...
    'state_min', state_min,...
    'state_max', state_max,...
    'input_min', input_min,...
    'input_max', input_max,...
    'dt', controller_sample_time,...
    'MPC_max_iters', MPC_max_iters,...
    'DSM_min', DSM_min,...
    'DSM_max', DSM_max,...
    'pathFG_kappa', pathFG_kappa,...
    'pathFG_max_iters', pathFG_max_iters,...
    'pathFG_max_N', pathFG_max_N,...
    'path', path...
);
params.path_size = size(path, 1);
params.deci_var_length = N * (n_x + n_u) + n_x;
params.quad_cost_mat = getQuadCostMat(params);
[params.lower_bound, params.upper_bound] = getDeciVarBounds(params);
params.lyapunov_thresh = (input_max(2) / k)^2;
params.first_init_guess = getFirstInitGuess(params);

function [lower_bound, upper_bound] = getDeciVarBounds(params)
    N = params.N;
    state_min = params.state_min;
    input_min = params.input_min;
    state_max = params.state_max;
    input_max = params.input_max;
    lower_bound = [repmat([state_min;input_min],N,1); state_min];
    upper_bound = [repmat([state_max;input_max],N,1); state_max];
end

% the matrix in the quadractic term of the cost function
function quad_cost_mat = getQuadCostMat(params)
    n_x = params.n_x;
    n_u = params.n_u;
    N = params.N;
    q = params.q;
    r_weight = params.r_weight;
    stage_diag = [q*ones(n_x,1); r_weight*ones(n_u,1)];
    quad_cost_mat_diag = [repmat(stage_diag,N,1); ones(n_x,1)];
    quad_cost_mat = diag(quad_cost_mat_diag);
end

function first_init_guess = getFirstInitGuess(params)

    n_x = params.n_x;
    n_u = params.n_u;
    dim = params.dim;
    N = params.N;
    pathFG_max_N = params.pathFG_max_N;
    path = params.path;
    path_size = params.path_size;
    deci_var_length = params.deci_var_length;

    first_init_guess = zeros(deci_var_length, 1);
    s_list = linspace(1, path_size, N+1);
    if N > pathFG_max_N
        % fill MPC states using path waypoints
        for i = 0:N
            s = s_list(i+1);
            ref = getRefFromPath(s, path);
            start_idx = i * (n_x + n_u) + 1;
            first_init_guess(start_idx:start_idx+dim-1) = ref;
        end
    else
        first_init_guess = zeros(deci_var_length, 1);
    end

end