function state_dot = unicycleContinuous(~, ctrl_input)
    angle = ctrl_input(1);
    state_dot = ctrl_input(2) * [cos(angle); sin(angle)];
end