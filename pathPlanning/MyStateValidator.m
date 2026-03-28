classdef MyStateValidator < nav.StateValidator & matlabshared.planning.internal.EnforceScalarHandle

    properties
        StateSpace;
        rect_obs;
    end

    methods

        function obj = MyStateValidator(state_space, params)
            obj@nav.StateValidator(state_space);
            obj.rect_obs = params.rect_obs;
        end

        function is_valid = isStateValid(obj, state)
            assert(false);
        end

        function [is_valid, last_valid] = isMotionValid(obj, state_1, state_2)

            dim = obj.StateSpace.NumStateVariables;
        
            is_valid = true;
            seg_dir = state_2 - state_1;
            seg_length = norm(seg_dir);
            seg_dir = seg_dir / seg_length;

            % check rectangular prism obstacles
            for i = 1:size(obj.rect_obs,1)
                inflated_min = obj.rect_obs(i,1:dim) - safety_margin_rect;
                inflated_max = obj.rect_obs(i,dim+1:2*dim) + safety_margin_rect;
                if obj.lineAndBoxCollision(state_1, seg_dir, seg_length, inflated_min, inflated_max)
                    is_valid = false;
                    last_valid = state_1;
                    break;
                end
            end

            if is_valid
                last_valid = state_2;
            end

        end

        function collided = lineAndBoxCollision(obj, seg_start, seg_dir, seg_length, box_min, box_max)

            t_enter = -inf;
            t_exit = inf;

            for i = 1:obj.StateSpace.NumStateVariables
                if abs(seg_dir(i)) < 1e-12
                    % Ray is parallel to this slab
                    if seg_start(i) < box_min(i) || seg_start(i) > box_max(i)
                        % Origin is outside the slab — no intersection
                        collided = false;
                        return;
                    end
                    % Otherwise the ray stays inside this slab for all t — skip
                else
                    t1 = (box_min(i) - seg_start(i)) / seg_dir(i);
                    t2 = (box_max(i) - seg_start(i)) / seg_dir(i);
                    
                    % t1 should be the near intersection, t2 the far
                    if t1 > t2
                        tmp = t1; t1 = t2; t2 = tmp;
                    end
                    
                    t_enter = max(t_enter, t1);
                    t_exit = min(t_exit, t2);
                    
                    if t_enter > t_exit
                        collided = false;
                        return;
                    end
                end
            end

            if t_exit < 0 || t_enter > seg_length
                collided = false;
            else
                collided = true;
            end
        end


    end

end