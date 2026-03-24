

%% Set Init values for decision variables

% Solver options:
% 1 - quadprog; 
% 2 - fmincon_x_u; 
% 3 - fmincon_u; 
% 4 - casadi_x_u; 
% 5 - fmincon_x_u_artificial_ref. 
% Note!!! In Simulink's MPC function, when solver = 3 or 5, change two places 
% 1) the solve value has to be set manually in Simulink MPC block, because they
% have different decision variable dimensions compared to other solvers. 
% Matlab check the dimension integrety using defaul solver which might
% cause discrepency.2) Make PathFG not working (PathFG_max_N = 0), this is
% done automatically below. 
solver = 2; % read the above NOTE first!

if(solver == 1 || solver == 2 || solver == 4)
    x_init = [start; 0; 0; 0; 0; 0; 0];
    u_init = [0; 0; 0; 0];
    decision_variables_init = repmat([x_init; u_init], prediction_horizon_MPC, 1);
    decision_variables_init = [decision_variables_init; x_init];
elseif(solver == 3)
    u_init = [0; 0; 0; 0];
    decision_variables_init = repmat(u_init, prediction_horizon_MPC, 1);
elseif(solver == 5)
    x_init = [start; 0; 0; 0; 0; 0; 0];
    u_init = [0; 0; 0; 0];
    decision_variables_init = repmat([x_init; u_init], prediction_horizon_MPC, 1);
    decision_variables_init = [decision_variables_init; x_init];
    decision_variables_init = [decision_variables_init; x_init(1:3)]; % Add artificial reference (x, y, z).
    PathFG_max_N = 0;
end


%% Set MPC Parameters

if(solver == 1 || solver == 2 || solver == 4)
    Q_R_P_weight_matrix = getQRPWeightMatrix(prediction_horizon_MPC, Q_MPC, R_MPC, LQR_P_matrix_full);
    Aeq = get_linear_equality_constraints_matrix(prediction_horizon_MPC, Q_MPC, R_MPC, A_discrete_outer, B_discrete_outer);
    lb = get_X_lower_bounds(prediction_horizon_MPC, Q_MPC, R_MPC, x_min, u_min);  % lower bounds
    ub = get_X_upper_bounds(prediction_horizon_MPC, Q_MPC, R_MPC, x_max, u_max);  % upper bounds

elseif(solver == 5)
    Q_R_P_weight_matrix = getQRPWeightMatrix(prediction_horizon_MPC, Q_MPC, R_MPC, LQR_P_matrix_full);
    Aeq = get_linear_equality_constraints_matrix(prediction_horizon_MPC, Q_MPC, R_MPC, A_discrete_outer, B_discrete_outer);
    Aeq = [Aeq; zeros(3, size(Aeq,2))]; % for artificial reference
    Aeq = [Aeq, zeros(size(Aeq,1), 3)];
    lb = get_X_lower_bounds(prediction_horizon_MPC, Q_MPC, R_MPC, x_min, u_min);  % lower bounds
    ub = get_X_upper_bounds(prediction_horizon_MPC, Q_MPC, R_MPC, x_max, u_max);  % upper bounds
    lb = [lb; x_min(1:3)]; % for artificial reference
    ub = [ub; x_max(1:3)]; % for artificial reference

elseif(solver == 3)
    
    lb = get_U_lower_bounds(prediction_horizon_MPC, Q_MPC, R_MPC, u_min);  % lower bounds
    ub = get_U_upper_bounds(prediction_horizon_MPC, Q_MPC, R_MPC, u_max);  % upper bounds

    A_hat = getAHat(A_discrete_outer, prediction_horizon_MPC);
    B_hat = getBHat(A_discrete_outer, B_discrete_outer, prediction_horizon_MPC);
    H_hat = getHHat(Q_MPC, LQR_P_matrix_full, prediction_horizon_MPC);
    R_hat = getRHat(R_MPC, prediction_horizon_MPC);
    
    H_MPC = B_hat' * H_hat * B_hat + R_hat;

    % adds 5 zeros to the auxiliary reference to state vector Gx
    Gx = [eye(3), zeros(3, 1);
          zeros(5, 4);
          zeros(1, 3), 1];
    
    W_MPC = [B_hat' * H_hat * A_hat, -B_hat' * H_hat * A_hat * Gx];

end


%% Get matrices for MPC_fmincon cost function

function A_hat = getAHat(A_discrete_outer, prediction_horizon_MPC)
    % Define matrix A_hat
    %[I;
    % A;
    % A^2;
    % .  
    % .  
    % .  
    % A^N];

    % Get the dimensions
    [rows_A, cols_A] = size(A_discrete_outer);

    % initialize the matrix
    A_hat = zeros(rows_A * (prediction_horizon_MPC + 1) , cols_A);

    % first rows
    A_hat(1 : rows_A, 1 :cols_A) = eye(rows_A);

    % Fill the rest
    for i = 1:prediction_horizon_MPC
        A_hat(i * rows_A + 1 : (i+1) * rows_A, ...
              1 : cols_A) = A_discrete_outer^i;
    end

    % A_hat = sparse(A_hat);
end


function B_hat = getBHat(A_discrete_outer, B_discrete_outer, prediction_horizon_MPC)
    % Define matrix B_hat
    %[0,   0,  ...   0;
    % B,   0,  ...   0;
    % AB,  B,  ...   0;
    % .    .    .    .  
    % .    .    .    .   
    % .    .    .    .  
    % A^(N-1)B,  A^(N-2)B,  ...  B];
   
    % Get the dimensions
    [rows_B, cols_B] = size(B_discrete_outer);

    % initialize the matrix
    B_hat = zeros(rows_B * (prediction_horizon_MPC + 1) , cols_B * prediction_horizon_MPC);

    % first row is 0; fill the rest with A and B based on N
    B_hat_row = zeros(rows_B, cols_B * prediction_horizon_MPC);
    for i = 0:(prediction_horizon_MPC - 1)
        for j = 0:i
            B_hat_row( : , cols_B * j + 1 : cols_B * (j + 1))...
                = A_discrete_outer^(i - j) * B_discrete_outer;
        end
        B_hat((i+1) * rows_B + 1 : (i+2) * rows_B,...
                1 : cols_B * prediction_horizon_MPC) = B_hat_row;
    end

    % B_hat = sparse(B_hat);
end


function H_hat = getHHat(Q_MPC, LQR_P_matrix_full, prediction_horizon_MPC)
    % Define matrix H_hat
    %[Q,   0,  ...   0,   0;
    % 0,   Q,  ...   0,   0;
    % 0,   0,  ...   0,   0;
    % .    .    .    .    .
    % .    .    .    .    .
    % 0    0   ...   Q    0
    % 0,   0,  ...   0,   P];
   
    
    % Get the dimensions
    [rows_Q, cols_Q] = size(Q_MPC);

    % initialize the matrix
    H_hat = zeros(rows_Q * (prediction_horizon_MPC + 1) , cols_Q * (prediction_horizon_MPC + 1));

    % fill the matrix
    for i = 1:prediction_horizon_MPC
        H_hat((i-1) * rows_Q + 1 : i * rows_Q, ...
              (i-1) * cols_Q + 1 : i * cols_Q) = Q_MPC;
    end

    % fill the last row
    H_hat(rows_Q * prediction_horizon_MPC + 1 : rows_Q * (prediction_horizon_MPC + 1), ...
        cols_Q * prediction_horizon_MPC + 1 : cols_Q * (prediction_horizon_MPC + 1)) = LQR_P_matrix_full;

    % H_hat = sparse(H_hat);
end


function R_hat = getRHat(R_MPC, prediction_horizon_MPC)
    % Define matrix R_hat
    %[R,   0,  ...   0;
    % 0,   R,  ...   0;
    % .    .   .     .
    % .    .     .   .
    % 0    0   ...   R];
   

    % Get the dimensions
    [rows_R, cols_R] = size(R_MPC);

    % initialize the matrix
    R_hat = zeros(rows_R * prediction_horizon_MPC , cols_R * prediction_horizon_MPC);

    % fill the matrix
    for i = 1:prediction_horizon_MPC
        R_hat((i-1) * rows_R + 1 : i * rows_R, ...
              (i-1) * cols_R + 1 : i * cols_R) = R_MPC;
    end

    % R_hat = sparse(R_hat);
end



%% Get matrices for MPC_quadprog cost function and constraints

function Q_R_P_weight_matrix = getQRPWeightMatrix(prediction_horizon_MPC, Q_MPC, R_MPC, LQR_P_matrix_full)
    % get weight_matrix:
    %[Q, 0, 0, 0, ..., 0;
    % 0, R, 0, 0, ..., 0;
    % 0, 0, Q, 0, ..., 0;
    % 0, 0, 0, R, ..., 0;
    % ...
    % 0, 0, 0, 0, ..., P];

    % Get the dimensions of Q and R
    [rows_Q, cols_Q] = size(Q_MPC);
    [rows_R, cols_R] = size(R_MPC);

    % matrix of Q's and R's
    Q_R_weight_matrix = [Q_MPC                , zeros(rows_Q, cols_R)    ;
                         zeros(rows_R, cols_Q), R_MPC                    ];
    Q_R_weight_matrices = kron(eye(prediction_horizon_MPC), Q_R_weight_matrix);
    [rows_Q_R, cols_Q_R] = size(Q_R_weight_matrices);

    % matrix of Q's, R's and P
    Q_R_P_weight_matrix = [Q_R_weight_matrices    , zeros(rows_Q_R, cols_Q);
                           zeros(rows_Q, cols_Q_R), LQR_P_matrix_full              ];

    % Old Code

    % % Create the weight matrix
    % Q_R_P_weight_matrix = zeros(rows_Q * (prediction_horizon_MPC + 1) + rows_R * prediction_horizon_MPC , cols_Q * (prediction_horizon_MPC + 1) + cols_R * prediction_horizon_MPC);
    % 
    % % Fill in the new matrix with Q and R based on prediction_horizon_MPC
    % for i = 1:prediction_horizon_MPC
    %     Q_R_P_weight_matrix((i-1)*(rows_R + rows_Q) + 1 : (i-1)*(rows_R + rows_Q) + rows_Q,...
    %         (i-1)*(cols_R + cols_Q) + 1 : (i-1)*(cols_R + cols_Q) + cols_Q) = Q_MPC;
    %     Q_R_P_weight_matrix((i-1)*(rows_R + rows_Q) + rows_Q + 1 : i*(rows_R + rows_Q),...
    %         (i-1)*(cols_R + cols_Q) + cols_Q + 1 : i*(cols_R + cols_Q)) = R_MPC;
    % end
    % 
    % % last row
    % Q_R_P_weight_matrix(prediction_horizon_MPC * (rows_R + rows_Q) + 1 : prediction_horizon_MPC * (rows_R + rows_Q) + rows_Q,...
    %     prediction_horizon_MPC * (cols_R + cols_Q) + 1 : prediction_horizon_MPC * (cols_R + cols_Q) + cols_Q) = LQR_P_matrix_full;

end


function linear_equality_constraints_matrix = get_linear_equality_constraints_matrix(prediction_horizon_MPC, Q_MPC, R_MPC, A_discrete_outer, B_discrete_outer)
    % Define matrix Aeq for linear equality constraints: Aeq * x = beq
    %[I,  0,  0,  0,  0, ..., 0,  0,  0;
    % A,  B, -I,  0,  0, ..., 0,  0,  0;
    % 0,  0,  A,  B, -I, ..., 0,  0,  0;
    % .   .   .   .   .  .    .   .   .
    % .   .   .   .   .   .   .   .   .
    % .   .   .   .   .    .  .   .   .
    % 0,  0,  0,  0,  0, ..., A,  B, -I];

    % Get the dimensions of Q and Q
    [rows_Q, cols_Q] = size(Q_MPC);
    [~, cols_R] = size(R_MPC);

    % initialize the matrix
    linear_equality_constraints_matrix = zeros(rows_Q * (prediction_horizon_MPC + 1) , cols_Q * (prediction_horizon_MPC + 1) + cols_R * prediction_horizon_MPC);

    % first rows
    linear_equality_constraints_matrix(1 : rows_Q,...
        1 :cols_Q) = eye(rows_Q);

    % Fill in the new matrix with A, B and -I based on prediction_horizon_MPC
    for i = 1:prediction_horizon_MPC
        linear_equality_constraints_matrix(i * rows_Q + 1 : (i+1) * rows_Q,...
            (i-1) * (cols_Q + cols_R) + 1 : (i-1) * (cols_Q + cols_R) + cols_Q) = A_discrete_outer;
        linear_equality_constraints_matrix(i * rows_Q + 1 : (i+1) * rows_Q,...
            (i-1) * (cols_Q + cols_R) + cols_Q + 1 : i * (cols_Q + cols_R)) = B_discrete_outer;        
        linear_equality_constraints_matrix(i * rows_Q + 1 : (i+1) * rows_Q,...
            i * (cols_Q + cols_R) + 1 : i * (cols_Q + cols_R) + cols_Q) = -eye(rows_Q);
    end
end

function lower_bounds = get_X_lower_bounds(prediction_horizon_MPC, Q_MPC, R_MPC, x_min, u_min)
    % Define lower bounds for X
    %[xmin;
    % umin-ueq;
    % xmin;
    % umin-ueq
    % .
    % .
    % .
    % xmin];

    % Get the dimensions of Q and Q
    [rows_Q, ~] = size(Q_MPC);
    [rows_R, ~] = size(R_MPC);

    % initialize the vector
    lower_bounds = zeros((rows_Q + rows_R) * prediction_horizon_MPC + rows_Q , 1);

    % Fill in the new vector based on prediction_horizon_MPC
    for i = 1:prediction_horizon_MPC
        lower_bounds((i-1) * (rows_Q + rows_R) + 1 : (i-1) * (rows_Q + rows_R) + rows_Q,...
            1) = x_min;
        lower_bounds((i-1) * (rows_Q + rows_R) + rows_Q + 1 : i * (rows_Q + rows_R),...
            1) = u_min;
    end

    % last rows
    lower_bounds(prediction_horizon_MPC * (rows_Q + rows_R) + 1 : prediction_horizon_MPC * (rows_Q + rows_R) + rows_Q,...
            1) = x_min;

end


function upper_bounds = get_X_upper_bounds(prediction_horizon_MPC, Q_MPC, R_MPC, x_max, u_max)
    % Define lower bounds for X
    %[xmax;
    % umax-ueq;
    % xmax;
    % umax-ueq
    % .
    % .
    % .
    % xmax];

    % Get the dimensions of Q and Q
    [rows_Q, ~] = size(Q_MPC);
    [rows_R, ~] = size(R_MPC);

    % initialize the vector
    upper_bounds = zeros((rows_Q + rows_R) * prediction_horizon_MPC + rows_Q , 1);

    % Fill in the new vector based on prediction_horizon_MPC
    for i = 1:prediction_horizon_MPC
        upper_bounds((i-1) * (rows_Q + rows_R) + 1 : (i-1) * (rows_Q + rows_R) + rows_Q,...
            1) = x_max;
        upper_bounds((i-1) * (rows_Q + rows_R) + rows_Q + 1 : i * (rows_Q + rows_R),...
            1) = u_max;
    end

    % last rows
    upper_bounds(prediction_horizon_MPC * (rows_Q + rows_R) + 1 : prediction_horizon_MPC * (rows_Q + rows_R) + rows_Q,...
            1) = x_max;

end