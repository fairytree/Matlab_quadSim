function [normalized_current_ref, next_ref_idx, s, lower_ref_idx]  = RRTWayPoint(...
    applied_reference, ...
    optimal_path, ...
    current_ref_idx,...
    ref_reached_threshold_rrt, ...
    smoothing_radius, ...
    last_ref_idx)

    % compute s for plotting
    if(current_ref_idx == last_ref_idx)
        s = 1;
    else
        ratio = norm(optimal_path(current_ref_idx, :)' - applied_reference(1:3))/norm(optimal_path(current_ref_idx, :)' - optimal_path(last_ref_idx, :)');
        s_reverse = current_ref_idx + ratio;
        s = size(optimal_path,1) - s_reverse + 1;
    end

    dist_to_current_ref = norm(optimal_path(current_ref_idx, :)' - applied_reference(1:3));
    if dist_to_current_ref < ref_reached_threshold_rrt && current_ref_idx > 1
        lower_ref_idx = current_ref_idx;
        next_ref_idx = current_ref_idx - 1;
    else
        lower_ref_idx = last_ref_idx;
        next_ref_idx = current_ref_idx;
    end

    normalized_current_ref = (optimal_path(current_ref_idx, :)' - applied_reference(1:3))/max(dist_to_current_ref,smoothing_radius);

end
 