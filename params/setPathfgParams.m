DSM_val_lower_threshold_pathFG = 0; % DSM_value target range lower bound
DSM_val_upper_threshold_pathFG = 0.01; % DSM_value target range upper bound
kappa_pathFG = 0.5;
iter_max_pathFG = 50; 
kappa_s = 2; % scaling factor for terminal thrust lyapunov threshold
kappa_o = 5; % the original paper use 20, scaling factor for terminal obstacle lyapunov threshold
PathFG_max_N = 30; % !!!PathFG is not included when prediction Horizon N >= 30