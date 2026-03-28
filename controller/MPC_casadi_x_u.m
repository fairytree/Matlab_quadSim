
%% NOTE: box obstacles not considered

function [deci_vars,...
    predicted_states_Nth_step,...
    desired_additional_thrust_and_body_rates]...
    = MPC_casadi_x_u(...
        prior_deci_vars,...
        x0,...
        v,...
        ssmodel,...
        weights,....
        obs_params,...
        agent_params,...
        constr_mat)
    
    % TODO kron function vs. for loop
    % TODO organization

    % set variables
    N = weights.N;
    K = weights.K;
    P = weights.P;
    Q = weights.Q;
    R = weights.R;
    agent_size = agent_params.agent_size;
    m = agent_params.m;
    g = agent_params.g;
    thrust_max = agent_params.thrust_max;
    thrust_min = agent_params.thrust_min;
    obs_p = obs_params.obs_p;
    obs_size = obs_params.obs_size;
    Q_R_P_weight_matrix = constr_mat.Q_R_P_weight_matrix;
    Aeq = constr_mat.Aeq;
    lb = constr_mat.lb;
    ub = constr_mat.ub;

    % variable dimensions
    [n_x, n_u] = size(ssmodel.B);
    xi = casadi.SX.sym('xi', n_x, N + 1);
    mu = casadi.SX.sym('mu', n_u, N);

    % indices of X that corresponds to xi and mu
    xi_idx = zeros(1, (N + 1) * n_x);
    for i = 0:N
        xi_idx(n_x*i+1:n_x*(i+1)) = (n_x+n_u)*i+1:(n_x+n_u)*i+n_x;
    end

    mu_idx = zeros(1, N * n_u);
    for i = 0:N-1
        mu_idx(n_u*i+1:n_u*(i+1)) = (n_x+n_u)*i+n_x+1:(n_x+n_u)*(i+1);
    end

    % decision variable
    n_X = (N + 1) * n_x + N * n_u;
    X = casadi.SX.sym('X', n_X, 1);
    X(xi_idx) = xi(:); % xi(:) converts xi to a matrix by concatenating columns of xi
    X(mu_idx) = mu(:);

    % current reference
    x_bar_v = [v(1:3)     ;...
               zeros(5, 1);...
               v(4)       ];

    % linear part of the cost function
    Q_x_bar_v = kron(ones(N, 1), [Q * x_bar_v; zeros(n_u, 1)]);
    Q_P_x_bar_v = [Q_x_bar_v  ;...
                   P * x_bar_v];

    % cost function
    cost_fcn = costFcn(X, Q_R_P_weight_matrix, Q_P_x_bar_v);

    % initial guess
    X_init = POSS(...
        prior_deci_vars,...
        K,...
        v,...
        ssmodel.A,...
        ssmodel.B);

    % equality constraints
    beq = getLinearEqualityConstraintsVector(N, Q, x0);
    eq_constr = Aeq * X - beq;

    % calculate the Lyapunov threshold value for computing the terminal set constraint
    P = getSparseP(P);
    [~, obstacle_V_bar] = obstacleDSM(obs_p, 0, x_bar_v, P, obs_size, agent_size, 0);
    [~, thrust_V_bar] = thrustDSM(thrust_max, thrust_min, m, g, 0, x_bar_v, K, P, 0);
    V_bar = min([obstacle_V_bar, thrust_V_bar]);

    % obstacle constraints as a function of X
    obs_constr = getObsContr(...
        X,...
        obs_p,...
        obs_size,...
        agent_size,...
        N,...
        n_x, ...
        n_u);

    % terminal constraints as a function of X
    term_constr = getTermConstr(X, n_x, x_bar_v, P, V_bar);

    % constraints but it's all packed together
    constr = [eq_constr  ;
              obs_constr ;
              term_constr];
    constr_lb = [zeros(size(Aeq, 1), 1)           ;
                 -inf * ones(numel(obs_constr), 1);
                 -inf                            ];
    constr_ub = [zeros(size(Aeq, 1), 1)     ;
                 zeros(numel(obs_constr), 1);
                 0                          ];

    % solving
    problem = struct('x', X, 'f', cost_fcn, 'g', constr);
    solver = casadi.nlpsol('solver', 'ipopt', problem);
    out = solver(...
        'x0', X_init,...
        'lbx', lb,...
        'ubx', ub,...
        'lbg', constr_lb,...
        'ubg', constr_ub);
    X_opt = full(out.x);
  
    % postprocessing
    desired_additional_thrust_and_body_rates = X_opt(10:13);
    predicted_states_Nth_step = [X_opt(end-8:end)];
    deci_vars = X_opt;

end


function cost = costFcn(X, Q_R_P, Q_P_x_bar_v)
    cost = 1/2 * X' * Q_R_P * X - Q_P_x_bar_v' * X;
end


% get sparse matrix
function P = getSparseP(P)
    
    for i = 1:numel(P)
        if abs(P(i)) < 1e-8
            P(i) = 0;
        end
    end
    P = sparse(P);

end


function linear_equality_constraints_vector = getLinearEqualityConstraintsVector(N, Q, x0)
    % Define vector beq for linear equality constraints: Aeq*x = beq
    %[x0;
    % 0;
    % 0;
    % .
    % .
    % .
    % 0];

     % Get the dimensions of Q and R
    [rows_Q, ~] = size(Q);

    % initialize the vector
    linear_equality_constraints_vector = zeros(rows_Q * (N + 1) , 1);

    % first rows
    linear_equality_constraints_vector(1 : rows_Q, 1) = x0;

    % Fill in the new vector zero vectors based on N
    for i = 1:N
        linear_equality_constraints_vector(i * rows_Q + 1 : (i+1) * rows_Q,...
            1) = zeros(rows_Q, 1);
    end

end


function obs_constr = getObsContr(...
    X,...
    obs_p,...
    obs_sizes,...
    agent_size,...
    N,...
    n_x, ...
    n_u)

    % initialization
    N_o = numel(obs_sizes);
    num_of_constr = N * N_o;
    obs_constr = casadi.SX.sym('obs_constr', num_of_constr, 1);

    for i = 1:N
        % p_start_idx and p_end_idx are used to locate
        % the position of the ith predicted state
        p_start_idx = (n_x + n_u) * i + 1;
        p_end_idx = (n_x + n_u) * i + 3;

        for j = 1:N_o
            % delta_p = position - jth obs position
            delta_p = X(p_start_idx:p_end_idx,1) - obs_p(:,j);

            % obstacle constraint functions
            obs_constr((i-1)*N_o+j,1)...
                = (agent_size + obs_sizes(1,j))^2 - delta_p' * delta_p;
        end
    end
    
end


function term_constr = getTermConstr(X, n_x, x_bar_v, P, V_bar)

    % terminal set constraint
    term_state = X(end-n_x+1:end,1);
    term_constr = lyapunovValue(term_state, x_bar_v, P) - V_bar;

end