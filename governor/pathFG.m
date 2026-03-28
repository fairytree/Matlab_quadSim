function [s, ref] = pathFG(prev_term_state, prev_s, prev_ref, params)

    path = params.path;
    path_size = params.path_size;
    DSM_min = params.DSM_min;
    DSM_max = params.DSM_max;

    if prev_s == path_size % goal reached
        s = prev_s;
        ref = prev_ref;
        return;
    end

    lower_s = prev_s;
    upper_s = path_size;
    s = upper_s;
    ref = getRefFromPath(s, path);
    DSM = termEnergyDSM(prev_term_state, ref, params);
    result = checkDSMFeasibility(DSM, DSM_min, DSM_max); 
    if result == 1 || result == 0 % goal is feasible
        return;
    end
    
    % bisection method
    iter = 0;
    while result ~= 0 && iter < pathFG_max_iters
        if result == -1 % DSM is negative
            upper_s = s;
            s = lower_s + pathFG_kappa * (upper_s - lower_s);
            ref = getRefFromPath(s, path);
        else
            lower_s = s;
            s = lower_s + pathFG_kappa * (upper_s - lower_s);
            ref = getRefFromPath(s, path);
        end
        DSM = termEnergyDSM(prev_term_state, ref, params);
        result = checkDSMFeasibility(DSM, DSM_min, DSM_max);
        iter = iter + 1;
    end

    if iter == pathFG_max_iters
        ref = prev_ref;
        disp("PathFG max iterations reached");
    end

end

% check if the DSM is within the target range
function result = checkDSMFeasibility(DSM, DSM_min, DSM_max)
    if DSM < DSM_min
        result = -1;
    elseif DSM > DSM_max
        result = 1;
    else
        result = 0;
    end
end
