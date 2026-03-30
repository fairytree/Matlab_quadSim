% N = 5;
N_list = [25, 5, 10]; % list of prediction horizons to compare
dim = 2; % dimension, either 2 or 3
n_x = 2; % number of states
n_u = 2; % number of control inputs
k = 0.5; % scaling factor for the terminal controller
q = 0.2; % scaling factor for stage cost
r_weight = 0.01; % scaling factor for input cost
MPC_max_iters = 10000;

colors = ["#0072BD",... % blue
        "#ffA500",...   % orange
        "#77AC30",...   % green
        "#D95319",...   % red
        "#4DBEEE",...   % light blue
        'black'];