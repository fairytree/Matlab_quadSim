function collided = lineSegmentAndBoxCollision(segment_start, ...
    segment_direction_normalized, segment_length, box_min, box_max)
    % Check if a line segment collides with an axis-aligned box (AABB).
    %
    % Uses the slab method (Kay-Kajiya):
    %   For each axis, compute the entry and exit t-values of the ray
    %   with the two slabs. The segment intersects the box if and only if
    %   the overall entry t is less than the overall exit t, and the
    %   intersection interval overlaps [0, segment_length].  
  
    %
    % Inputs:
    %   segment_start                - 1x3 or 3x1, start point of segment
    %   segment_direction_normalized - 1x3 or 3x1, unit direction vector
    %   segment_length               - scalar, length of the segment
    %   box_min                      - 1x3 or 3x1, [x_min, y_min, z_min]
    %   box_max                      - 1x3 or 3x1, [x_max, y_max, z_max]
    %
    % Output:
    %   collided - true if the segment intersects the box, false otherwise
    
    % the line segment equation:
    % v(t) = v_start + t * dir  (1) direction is unit-length, t is [0, length]
    % 
    % collide if at least one point on the line segment is inside the box   
    % box_min <= v(t) <= box_max    (2)
    % or equivalently
    % x_min <= v_x(t) <= x_max && y_min <= v_y(t) <= y_max && z_min <= v_z(t) <= z_max

    % plug (1) into (2), we get
    % [(box_min - v_start) / dir] <=  t  <= [(box_max - v_start)/ dir]  (3)

    % Ensure column vectors
    segment_start = segment_start(:)';
    segment_direction_normalized = segment_direction_normalized(:)';
    box_min = box_min(:)';
    box_max = box_max(:)';
    
    t_enter = -inf;
    t_exit  =  inf;
    
    % iterate 3 axes, as for a point to be inside box, all three axes need to
    % satisfy the above equation (2). 
    for i = 1:length(segment_start)
        if abs(segment_direction_normalized(i)) < 1e-12
            % Ray is parallel to any axis
            if segment_start(i) < box_min(i) || segment_start(i) > box_max(i)
                % Origin is outside the slab — no intersection
                collided = false;
                return;
            end       
        else
            % see above equation (3)
            t1 = (box_min(i) - segment_start(i)) / segment_direction_normalized(i);
            t2 = (box_max(i) - segment_start(i)) / segment_direction_normalized(i);
            
            % t1 should be the near intersection, t2 the far
            if t1 > t2
                tmp = t1; t1 = t2; t2 = tmp;
            end
            
            % t needs to be the intersection of all intervals of the 3 axes
            t_enter = max(t_enter, t1);   % the largest of all the 3 axes
            t_exit  = min(t_exit,  t2);   % the smallest of all the 3 axes
            
            if t_enter > t_exit
                collided = false;
                return;
            end
        end
    end
    
    % The ray intersects the box in the interval [t_enter, t_exit],
    % and t needs to be in the range of [0, segment_length].
    if t_exit < 0 || t_enter > segment_length
        collided = false;
    else
        collided = true;
    end
end
