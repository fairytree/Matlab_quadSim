if path_planner == "RRT*"
    % RRT; % stop when goal reached
    RRT2;  % stop when time out
elseif path_planner == "Pot. Field"
    potentialField;
end