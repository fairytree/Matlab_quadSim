function [angular_rate_DSM, angular_rate_lyapunov_threshold] =...
    angularRateDSM(x_bar_v, lyapunov_value, K, P, u_max, u_min)

    % initalize Lyapunov thresholds
    omega_max_thresholds = zeros(3, 1);
    omega_min_thresholds = zeros(3, 1);

    % disp(lyapunov_value);

    % loop over omega_x, omega_y and omega_z
    for omega_idx = 1:3
        % Lyapunov threshold for omega_max
        c_omega_max = -K(omega_idx+1,:)';
        d_omega_max = u_max(omega_idx+1) - K(omega_idx+1,:) * x_bar_v;
        omega_max_threshold = lyapunovThreshold(x_bar_v, d_omega_max, c_omega_max, P);
        omega_max_thresholds(omega_idx) = omega_max_threshold;

        % Lyapunov threshold for omega_min
        c_omega_min = K(omega_idx+1,:)';
        d_omega_min = K(omega_idx+1,:) * x_bar_v - u_min(omega_idx+1);
        omega_min_threshold = lyapunovThreshold(x_bar_v, d_omega_min, c_omega_min, P);
        omega_min_thresholds(omega_idx) = omega_min_threshold;
    end

    % compute the total Lyapunov threshold
    omega_max_lyapunov_threshold = min(omega_max_thresholds);
    omega_min_lyapunov_threshold = min(omega_min_thresholds);
    angular_rate_lyapunov_threshold = min([omega_max_lyapunov_threshold, omega_min_lyapunov_threshold]);

    % compute the DSM
    angular_rate_DSM = angular_rate_lyapunov_threshold - lyapunov_value;

end