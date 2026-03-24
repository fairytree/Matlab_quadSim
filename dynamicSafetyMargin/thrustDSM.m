function [DSM_s, thrust_lyapunov_threshold] = thrustDSM( ...
    thrust_max, ...
    thrust_min, ...
    m, ...
    g, ...
    lyapunov_value, ...
    v, ...
    K, ...
    P, ...
    kappa_s)
    % Gets the dynamic safety margin of the saturation constraint, $\Delta^s_i$.
    % Args:
    %     thrust_max: maximum thrust
    %     thrust_min: minimum thrust
    %     mass_for_controller: mass of quadrotor
    %     g: gravitational acceleration
    %     k_P: proportional gain for outer loop controller
    %     k_D: derivative gain for outer loop controller
    %     epsilon: a constant between 0 and 1
    %     x: current state
    %     v: current applied reference
    %     P: used for calculating the Lyapunov threshold value
    %     kappa_s: a positive scaling factor

    % % Original Paper: Compute threshold value separately for thrust_min and thrust_max
    % gamma_T_max = (thrust_max - m * g)^2 * (k_P + epsilon * (1 - epsilon) * k_D^2) / (2 * mass_for_controller^2 * (k_P^2 + k_D^2 * (k_P + epsilon * k_D^2 - 2 * epsilon * k_P)));
    % gamma_T_min = (thrust_min - m * g)^2 * (k_P + epsilon * (1 - epsilon) * k_D^2) / (2 * mass_for_controller^2 * (k_P^2 + k_D^2 * (k_P + epsilon * k_D^2 - 2 * epsilon * k_P)));
    
    % % Self-proof: Compute threshold value separately for thrust_min and thrust_max
    % gamma_T_max = (thrust_max - m * g)^2 / (K(1,1:9) * inv(P) * K(1, 1:9)');
    % gamma_T_min = (thrust_min - m * g)^2 / (K(1,1:9) * inv(P) * K(1, 1:9)');
    
    % self-proof linear constraint
    c_Tmin = K(1,1:9)';
    d_Tmin = -thrust_min + m * g + K(1,1:9) * v;
    lyapunov_threshold_Tmin = lyapunovThreshold(v, d_Tmin, c_Tmin, P);
        
    c_Tmax = -K(1,1:9)';
    d_Tmax = thrust_max - m * g - K(1,1:9) * v;
    lyapunov_threshold_Tmax = lyapunovThreshold(v, d_Tmax, c_Tmax, P);

    thrust_lyapunov_threshold = min([lyapunov_threshold_Tmax, lyapunov_threshold_Tmin]);
    DSM_s = kappa_s * (thrust_lyapunov_threshold - lyapunov_value);

end