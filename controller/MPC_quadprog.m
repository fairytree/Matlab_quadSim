
function [decision_variables,...
    predicted_states_Nth_step,...
    desired_additional_thrust_and_body_rates] = MPC_quadprog(...
    prior_deci_vars,...
    x0,...
    v,...
    ssmodel,...
    weights,...
    constr_mat)
    % NOTE: MPC_quadprog uses both x and u as decision variables.
    
    % set variables
    N = weights.N;
    K = weights.K;
    P = weights.P;
    Q = weights.Q;
    R = weights.R;
    Q_R_P_weight_matrix = constr_mat.Q_R_P_weight_matrix;
    Aeq = constr_mat.Aeq;
    lb = constr_mat.lb;
    ub = constr_mat.ub;
    
    % Define the objective function to minimize
    Xref = get_Xref(N, Q, v);
    Q_P_Xref_weight_matrix = get_Q_P_Xref_weight_matrix(N, Q, R, P, Xref);
    cost_func = @(X) 1/2 * X' * Q_R_P_weight_matrix * X - Q_P_Xref_weight_matrix' * X;

    % Get initial guess
    initial_guess_X = POSS(prior_deci_vars,...
                            K,...
                            v,...
                            ssmodel.A,...
                            ssmodel.B);

    % set the inequal constraints
    A = [];
    b = [];

    % set the equal constraints
    beq = get_linear_equality_constraints_vector(N, Q, x0);

    nonlcon = [];

    % Call solver to perform the optimization with options
    options = optimoptions('quadprog','Display','iter', 'Algorithm','active-set');
    X_opt = quadprog(Q_R_P_weight_matrix,-Q_P_Xref_weight_matrix,A,b,Aeq,beq,lb,ub,initial_guess_X,options);
    
    desired_additional_thrust_and_body_rates = [X_opt(10); X_opt(11); X_opt(12); X_opt(13)]; 
    predicted_states_Nth_step = [X_opt(end-8 : end)];
    decision_variables = X_opt(1:size(prior_deci_vars,1),1);
    
end


function Xref = get_Xref(N, Q, v)
    x_ref = ...
                [v(1,1);...
                 v(2,1);...
                 v(3,1);...
                 0.0;...
                 0.0;...
                 0.0;...
                 0.0;...
                 0.0;...
                 v(4,1)]; % x, y, z, x_dot, y_dot, z_dot, roll, pitch, yaw
           
    % initialize Xref
    Xref = zeros((N + 1) * size(Q,1) , 1);
        for i = 1:(N+1)  
            Xref((i-1) * size(Q,1) + 1 : i * size(Q,1), 1) = x_ref;
        end     
end


function Q_P_Xref_weight_matrix = get_Q_P_Xref_weight_matrix(N, Q, R, P, Xref)
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
    Q_P_Xref_weight_matrix = zeros(rows_Q * (N + 1) + rows_R * N , 1);

    % Fill in the new matrix with R and Q based on N
    for i = 1:N
        Q_P_Xref_weight_matrix((i-1)*(rows_R + rows_Q) + 1 : (i-1)*(rows_R + rows_Q) + rows_Q,...
            1) = Q * Xref((i-1) * rows_Q + 1 : i * rows_Q);
        Q_P_Xref_weight_matrix((i-1)*(rows_R + rows_Q) + rows_Q + 1 : i*(rows_R + rows_Q),...
            1) = zeros(rows_R, 1);
    end

    % last row of P
    Q_P_Xref_weight_matrix(N * (rows_R + rows_Q) + 1 : N * (rows_R + rows_Q) + rows_Q,...
        1) = P * Xref(N * rows_Q + 1 : (N+1) * rows_Q);

end


function linear_equality_constraints_vector = get_linear_equality_constraints_vector(N, Q, x0)
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
