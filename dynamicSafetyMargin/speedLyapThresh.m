function speed_lyap_thresh = speedLyapThresh(~, params)
    k = params.k; % scaling factor for terminal controller
    max_speed = params.input_max(2);
    speed_lyap_thresh = (max_speed / k)^2;
end