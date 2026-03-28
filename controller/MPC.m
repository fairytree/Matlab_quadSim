function [opt_deci_var, term_pred_state] = MPC(curr_state, ref, params)

    n_x = params.n_x;
    quad_cost_mat = params.quad_cost_mat;
    lower_bound = params.lower_bound;
    upper_bound = params.upper_bound;
    max_iters = params.max_iters;

    ref_state = ref;
    [A, b, A_eq, b_eq] = getLinConstr();
    nonlin_constr_func = @(deci_var) getNonlinConstr(deci_var, curr_state, ref_state, params);
    lin_cost_mat = getLinCostMat(ref_state, params);
    cost_with_grad_func = @(deci_var) getCostWithGrad(deci_var, quad_cost_mat, lin_cost_mat);
    init_guess = getInitGuess(curr_state, params);

    options = optimoptions(...
        'fmincon',...
        Algorithm='interior-point',...
        MaxIterations=max_iters,...
        Display='notify-detailed',...
        MaxFunctionEvaluations=100000 ...
    );
    opt_deci_var = fmincon(...
        cost_with_grad_func,...
        init_guess,...
        A, b, A_eq, b_eq,...
        lower_bound, upper_bound,...
        nonlin_constr_func,...
        options...
    );
    term_pred_state = opt_deci_var(end-n_x+1:end);

end

% get cost function with gradient
function [cost, cost_grad] = getCostWithGrad(deci_var, quad_cost_mat, lin_cost_mat) 
    cost = 0.5 * deci_var' * quad_cost_mat * deci_var - lin_cost_mat' * deci_var;
    cost_grad = quad_cost_mat * deci_var - lin_cost_mat;
end

function [A, b, A_eq, b_eq] = getLinConstr()
    A = [];
    b = [];
    A_eq = [];
    b_eq = [];
end

% nonlinear inequality and equality constraints
function [nonlin_constr, nonlin_eq_constr] = getNonlinConstr(deci_var, curr_state, ref_state, params)

    N = params.N;
    n_x = params.n_x;
    n_u = params.n_u;
    rectangular_obs = params.rectangular_obs;
    buffer = params.buffer;
    dim = params.dim;
    lyapunov_thresh = params.lyapunov_thresh;

    n_o = size(rectangular_obs, 1);
    constr_count = N * n_o + 1;
    nonlin_constr = zeros(constr_count, 1);
    eq_constr_count = (N + 1) * n_o;
    nonlin_eq_constr = zeros(eq_constr_count, 1);

    % obstacle constrarints
    for i = 0:N-1 % loop over predicted states
        start_idx = i * (n_x + n_u) + 1;
        pos = deci_var(start_idx:start_idx+dim-1);
        for j = 1:n_o % loop over obstacles
            max_dist = -Inf;
            for k = 1:dim % loop over dimensions
                dist = max(rectangular_obs(j,k)-pos(k), pos(k)-rectangular_obs(j,k+dim));
                if dist > max_dist
                    max_dist = dist;
                end
            end
            nonlin_constr(i*n_o+j) = -max_dist + buffer;
        end
    end
    % terminal set constraint
    term_state = TODO_USE_THE_FUNC;

    % initial condition
    nonlin_eq_constr(1:n_x) = deci_var(1:n_x) - curr_state;
    % system model constraints
    for i = 1:N
        curr_start_idx = i * (n_x + n_u) + 1;
        prev_start_idx = curr_start_idx - n_x - n_u;
        curr_pred_state = deci_var(curr_start_idx:curr_start_idx+n_x-1); % predicted state from timestep i (xi_i)
        prev_pred_state = deci_var(prev_start_idx:prev_start_idx+n_x-1); % predicted state from timestep i-1 (xi_{i-1})
        prev_pred_input = deci_var(prev_start_idx+n_x:curr_start_idx-1); % predicted control input (mu_{i-1})
        nonlin_eq_constr(i*n_x+1:(i+1)*n_x) = getNextState(prev_pred_state, prev_pred_input) - curr_pred_state;
    end

end

function init_guess = getInitGuess(curr_state, params)
    N = params.N;
    n_u = params.n_u;
    repeated_part = [curr_state; zeros(n_u,1)];
    init_guess = [repmat(repeated_part,N,1); curr_state];
end

% predict the state on the next timestep using the system equations
function next_state = getNextState(curr_state, ctrl_input)
    angle = ctrl_input(1);
    speed = ctrl_input(2);
    next_state = curr_state + speed * [cos(angle); sin(angle)];
end

% stage cost = q * norm(x - r)^2
% terminal cost = norm(x - r)^2
% if you expand these costs, you get a quadratic term and a linear term

% the matrix in the linear term of the cost function (it's actually a vector)
% this matrix is dependent on the reference state
function lin_cost_mat = getLinCostMat(ref_state, params)
    N = params.N;
    n_u = params.n_u;
    q = params.q;
    repeated_part = [q*ref_state; zeros(n_u,1)];
    lin_cost_mat = [repmat(repeated_part,N,1); ref_state];
end
