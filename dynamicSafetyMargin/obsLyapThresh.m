% Lyapunov threshold value for the obstacle constraints
function obs_lyap_thresh = obsLyapThresh(ref_state, params)
    rect_obs = params.rect_obs;
    obs_count = size(rect_obs, 1);
    lyap_threshes = zeros(obs_count, 1);
    for i = 1:obs_count
        squared_dist = 0;
        for j = 1:dim
            curr_dim_dist = max(...
                rect_obs(i,j)-ref_state(j),...
                ref_state(j)-rect_obs(i,j+dim)...
            );
            squared_dist = squared_dist + curr_dim_dist^2;
        end
        lyap_threshes(i) = squared_dist;
    end
    obs_lyap_thresh = min(lyap_threshes);
end

