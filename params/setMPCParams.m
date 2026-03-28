N = [5]; % prediction horizon(s)
dim = 2; % dimension, either 2 or 3
n_x = 2; % number of states
n_u = 2; % number of control inputs
k = 1; % scaling factor for the terminal controller
q = 1; % scaling factor for stage cost
MPC_max_iters = 5000;

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