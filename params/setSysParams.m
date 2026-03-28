start = [0; 0];
goal = [4; 7];
init_state = start;

state_min = [-Inf; -Inf];
state_max = -x_min;

input_min = [-2*pi; 0];
input_max = [2*pi; 0.3];

controller_sample_time = 1/200;
continous_sample_time = 1/1000;
pathFG_sample_time = controller_sample_time;

sim_ = 18;
set_param('sim', 'StopTime', num2str(sim_));