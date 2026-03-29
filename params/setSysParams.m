start = [0; 0];
goal = [4; 7];
init_state = start;

state_min = [-Inf; -Inf];
state_max = -state_min;

input_min = [-2*pi; 0];
input_max = [2*pi; 3.0];

controller_sample_time = 1/10;
continous_sample_time = 1/1000;
pathFG_sample_time = controller_sample_time;

sim_duration = 18;
set_param('Sim_unicycle', 'StopTime', num2str(sim_duration));