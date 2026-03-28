
%% NOTE: box obstacles not considered

function [deci_vars,...
    predicted_states_Nth_step,...
    desired_additional_thrust_and_body_rates,...
    iterations]...
    = MPC_fmincon_x_u(...
        prior_deci_vars,...
        x0,...
        v,...
        ssmodel,...
        weights,....
        obs_params,...
        agent_params,...
        constr_mat, ...
        safety_margin_universal, ...
        MPC_max_iters, ...
        path)

    % NOTE: MPC_fmincon_x_u uses both xi and mu as decision variables.
    % IDEA: define n_x = size(ssmodel.A, 1) and n_u = size(ssmodel.B, 2)
    
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

    % calculate the Lyapunov threshold value for computing the terminal set constraint
    P = getSparseP(P);
    x_bar_v = [v(1:3); 0; 0; 0; 0; 0; v(4)];
    [~, obstacle_V_bar] = obstacleDSM(obs_p, 0, x_bar_v, P, obs_size, agent_size, 0);
    [~, thrust_V_bar] = thrustDSM(thrust_max, thrust_min, m, g, 0, x_bar_v, K, P, 0);
    [~, angular_rate_V_bar] = angularRateDSM(x_bar_v, 0, K, P, ub(size(ssmodel.A,1)+1:size(ssmodel.A,1)+size(ssmodel.B,2)), lb(size(ssmodel.A,1)+1:size(ssmodel.A,1)+size(ssmodel.B,2)));
    V_bar = min([obstacle_V_bar, thrust_V_bar, angular_rate_V_bar]);

    % set initial guess
    initial_guess_X = POSS(prior_deci_vars,...
        K,...
        v,...
        ssmodel.A,...
        ssmodel.B);

    % set the inequal constraints
    A = [];
    b = [];

    % set the equal constraints
    beq = getLinearEqualityConstraintsVector(N, Q, x0);

    % get nonlinear constraints with gradient
    nonlcon = @(x) getNonlConWithGrad(...
        x,...
        obs_p,...
        obs_size,...
        agent_size,...
        N,...
        size(ssmodel.A, 1), ...
        size(ssmodel.B, 2),...            
        P,...
        V_bar,...
        x_bar_v,...
        safety_margin_universal);

    % get Hessian for Lagrangian 
    lag_hess = @(x,lambda) getLagrangianHess(...
        x,...
        lambda,...
        Q_R_P_weight_matrix,...
        N,...
        size(ssmodel.A, 1),...
        size(ssmodel.B, 2),...
        numel(obs_size),...
        P);

    % get Q_P_Xref_vector
    Xref = get_Xref(N, Q, x_bar_v);
    Q_P_Xref_vector = get_Q_P_Xref_vector(N, Q, R, P, Xref);
   
    % make sparse matrices
    [Aeq, beq, Q_P_Xref_vector, Q_R_P_weight_matrix] = ...
        getSparseMatrices(Aeq, beq, Q_P_Xref_vector, Q_R_P_weight_matrix);

    % get cost function with gradient
    cost_func_with_grad = @(X) getCostFunctionWithGradient(X, Q_R_P_weight_matrix, Q_P_Xref_vector);
  
    % Call fmincon to perform the optimization with options (interior-point
    % or sqp) (option: SubproblemAlgorithm="cg", EnableFeasibilityMode=true)
    options = optimoptions('fmincon',...
        Algorithm='interior-point',...
        MaxIterations=MPC_max_iters,...
        SpecifyObjectiveGradient=true,...
        SpecifyConstraintGradient=true,...
        HessianFcn=lag_hess,...
        Display='notify-detailed'); % 'notify-detailed' displays output when fmincon doesn't converge
    [X_opt, ~, exit_status, fmincon_output] = fmincon(cost_func_with_grad, initial_guess_X, A, b, Aeq, beq, lb, ub, nonlcon, options);
  
    % output
    desired_additional_thrust_and_body_rates = X_opt(10:13); 
    predicted_states_Nth_step = X_opt(end-8:end);
    deci_vars = X_opt(1:size(prior_deci_vars,1),1);
    iterations = fmincon_output.iterations;

    % display the current state if a solution is not found
    if not(exit_status == 1)
        state_constr_violation = false;

        % check if the current state is out-of-bounds
        for state_idx = 1:size(ssmodel.A,1)
            if not((lb(state_idx) < x0(state_idx)) && (x0(state_idx) < ub(state_idx)))
                disp("The current state is out-of-bounds");
                state_constr_violation = true;
                break;
            end
        end
        if not(state_constr_violation)
            disp("========================");
            disp("CONSTRAINTS NOT VIOLATED");
            disp("========================")
        end

        % convert the format so that it can be directly copied and pasted
        x0_str = "Current state: [";
        if numel(x0) > 1
            for state_idx = 1:numel(x0)-1
                x0_str = strcat(x0_str, num2str(x0(state_idx)), "; ");
            end
        end
        x0_str = strcat(x0_str, num2str(x0(end)), "]");
        disp(x0_str);
    end

end


function Xref = get_Xref(N, Q, x_bar_v)
    % initialize Xref
    Xref = zeros((N + 1) * size(Q,1) , 1);
    for i = 1:(N+1)
        Xref((i-1) * size(Q,1) + 1 : i * size(Q,1), 1) = x_bar_v;
    end
end


function Q_P_Xref_vector = get_Q_P_Xref_vector(N, Q, R, P, Xref)
    % get weight_matrix:
    %[Q * Xref(0 * rows_Q + 1:rows_Q);
    % zeros(rows_R, 1);
    % Q * Xref(1 * rows_Q + 1 : 2 * rows_Q);
    % zeros(rows_R, 1);
    % ...
    % P * Xref(N * rows_Q + 1 : (N + 1) rows_Q)];

    % Get the dimensions of Q and R
    [rows_Q, ~] = size(Q);
    [rows_R, ~] = size(R);

    % Create the weight matrix
    Q_P_Xref_vector = zeros(rows_Q * (N + 1) + rows_R * N , 1);

    % Fill in the new matrix with R and Q based on N
    for i = 1:N
        Q_P_Xref_vector((i-1)*(rows_R + rows_Q) + 1 : (i-1)*(rows_R + rows_Q) + rows_Q,...
            1) = Q * Xref((i-1) * rows_Q + 1 : i * rows_Q);
        Q_P_Xref_vector((i-1)*(rows_R + rows_Q) + rows_Q + 1 : i*(rows_R + rows_Q),...
            1) = zeros(rows_R, 1);
    end

    % last row of P
    Q_P_Xref_vector(N * (rows_R + rows_Q) + 1 : N * (rows_R + rows_Q) + rows_Q,...
        1) = P * Xref(N * rows_Q + 1 : (N+1) * rows_Q);
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


% get sparse matrices
function  [Aeq, beq, Q_P_Xref_vector, Q_R_P_weight_matrix] = ...
        getSparseMatrices(Aeq, beq, Q_P_Xref_vector, Q_R_P_weight_matrix)

    Aeq = sparse(Aeq);
    beq = sparse(beq);
    Q_P_Xref_vector = sparse(Q_P_Xref_vector);
    Q_R_P_weight_matrix = sparse(Q_R_P_weight_matrix);

end


% get cost function with gradient
function [f, gradf] = getCostFunctionWithGradient(X, Q_R_P_weight_matrix, Q_P_Xref_vector) 
    
    f = 1/2 * X' * Q_R_P_weight_matrix * X - Q_P_Xref_vector' * X;
    gradf = Q_R_P_weight_matrix * X - Q_P_Xref_vector;

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


function [nonlcon, nonlconeq, grad_nonlcon, grad_nonlconeq] = getNonlConWithGrad(...
    x,...
    obs_p,...
    obs_size,...
    agent_size,...
    N,...
    n_x, ...
    n_u,...
    P_matrix,...
    V_bar,...
    x_bar_v,...
    safety_margin_universal)
    % note: n_x is the dimension of the outer-loop state, and n_u is the
    % dimension of the outer-loop control input

    % nonlinear equality constraints
    nonlconeq = [];
    grad_nonlconeq = [];

    % nonlinear inequality constraints
    N_o = numel(obs_size);
    num_of_constraints = N * N_o + 1;
    nonlcon = zeros(1, num_of_constraints);

    % obstacle constrarints
    for i = 1:N
        % position_start_idx and position_end_idx are used to locate
        % the position of the ith predicted state
        position_start_idx = (n_x + n_u) * i + 1;
        position_end_idx = (n_x + n_u) * i + 3;

        for j = 1:N_o
            % delta_p is the position minus the obstacle center
            delta_p = x(position_start_idx:position_end_idx,1) - obs_p(:,j);

            % obstacle constraint functions
            nonlcon(1,(i-1)*N_o+j)...
                = (agent_size + obs_size(1,j) + safety_margin_universal)^2 - delta_p' * delta_p;
        end
    end

    % terminal set constraint
    term_state_start_idx = (n_x + n_u) * N + 1;
    term_state_end_idx = (n_x + n_u) * N + n_x;
    term_state = x(term_state_start_idx:term_state_end_idx,1);

    nonlcon(1,num_of_constraints)...
        = lyapunovValue(term_state, x_bar_v, P_matrix) - V_bar;
    

    % nonlinear inequality constraints gradient
    grad_nonlcon = zeros(numel(x), num_of_constraints);

    for i = 1:N
        position_start_idx = (n_x + n_u) * i + 1;
        position_end_idx = (n_x + n_u) * i + 3;
        for j = 1:N_o
            grad_nonlcon(position_start_idx:position_end_idx,(i-1)*N_o+j)...
                = 2 * (obs_p(:,j) - x(position_start_idx:position_end_idx,1));
        end
    end

    % terminal set constraint gradient
    grad_nonlcon(term_state_start_idx:term_state_end_idx,num_of_constraints)...
        = 2 * P_matrix * term_state - 2 * P_matrix * x_bar_v;
end


% calculates the Hessian of the Lagrangian    
function hess = getLagrangianHess(x, lambda, Q_R_P_weight_matrix, N, n_x, n_u, N_o, P)
  
    % objective function Hessian
    hess = Q_R_P_weight_matrix;

    % obstacle constraints Hessian
    for i = 1:N
        position_start_idx = (n_x + n_u) * i + 1;
        position_end_idx = (n_x + n_u) * i + 3;

        ith_step_constraint_hess = [zeros(position_start_idx - 1, (n_x + n_u) * N + n_x);...
            zeros(3, position_start_idx - 1), -2 * eye(3), zeros(3, (n_x + n_u) * N + n_x - position_end_idx);...
            zeros((n_x + n_u) * N + n_x - position_end_idx, (n_x + n_u) * N + n_x)];
        for j = 1:N_o
            hess = hess + lambda.ineqnonlin((i-1)*N_o+j) * ith_step_constraint_hess;
        end
    end

    % terminal set constraint Hessian
    before_term_state_idx = (n_x + n_u) * N;
    term_hess = [zeros(before_term_state_idx, numel(x))   ;
                 zeros(n_x, before_term_state_idx), P' + P];
    hess = hess + lambda.ineqnonlin(N*N_o+1) * term_hess;
end