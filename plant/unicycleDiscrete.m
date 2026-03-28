function next_state = unicycleDiscrete()
    sample_time = params.controller_sample_time;
    angle = ctrl_input(1);
    speed = ctrl_input(2);
    next_state = state + sample_time * speed * [cos(angle); sin(angle)];
end