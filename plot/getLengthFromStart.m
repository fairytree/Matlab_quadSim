function length_from_start = getLengthFromStart(s, path)

    % get the last waypoint and its respective index
    last_waypoint_idx = floor(s);
    last_waypoint = path(last_waypoint_idx,:)';

    % initialize length_from_start
    length_from_start = 0;

    % calculate the length from the start to last_waypoint
    if last_waypoint_idx > 1
        last_last_waypoint_idx = last_waypoint_idx - 1;        
    
        for waypoint_idx = 1:last_last_waypoint_idx
            length_from_start = length_from_start + norm(path(waypoint_idx+1,:) - path(waypoint_idx,:));
        end
    end
    
    v = getVPosition(s, path); % get the current auxiliary reference    
    length_from_last_waypoint = norm(v(1:3) - last_waypoint); % calculate the length from the last waypoint
    length_from_start = length_from_start + length_from_last_waypoint;

end