function collided = lineAndBoxCollision(segment_start, ...
    segment_direction_normalized, segment_length, box_min, box_max)

    t_enter = -inf;
    t_exit  =  inf;

    for i = 1:3
        if abs(segment_direction_normalized(i)) < 1e-12
            % Ray is parallel to this slab
            if segment_start(i) < box_min(i) || segment_start(i) > box_max(i)
                % Origin is outside the slab — no intersection
                collided = false;
                return;
            end
            % Otherwise the ray stays inside this slab for all t — skip
        else
            t1 = (box_min(i) - segment_start(i)) / segment_direction_normalized(i);
            t2 = (box_max(i) - segment_start(i)) / segment_direction_normalized(i);
            
            % t1 should be the near intersection, t2 the far
            if t1 > t2
                tmp = t1; t1 = t2; t2 = tmp;
            end
            
            t_enter = max(t_enter, t1);
            t_exit  = min(t_exit,  t2);
            
            if t_enter > t_exit
                collided = false;
                return;
            end
        end
    end

    if t_exit < 0 || t_enter > segment_length
        collided = false;
    else
        collided = true;
    end
end
