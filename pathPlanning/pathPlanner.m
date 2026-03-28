switch path_planner
case 1
    state_space = nav.StateSpace('state_space', dim, [state_min, state_max]);
    state_validator = nav.StateValidator(state_space);
case 2
    potentialField;
end