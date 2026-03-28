init_ref = init_state;
DSM_min = 0; % DSM value target range lower bound
DSM_max = 0.01; % DSM value target range upper bound
pathFG_kappa = 0.5; % used in bisection method
pathFG_max_iters = 50;
% kappa_s = 2; % scaling factor for speed DSM
% kappa_o = 5; % scaling factor for obstacle DSM
pathFG_max_N = 30; % max prediction horizon where PathFG is still used