function gamma_value = lyapunovThreshold(v, d, c, P)
    % Gets the Lyapunov threshold value, Gamma(v), from the applied reference v.
    % The constraint is c' * x <= d(v), and the Lyapunov function is
    % lyapunovValue(x, v) = (x - v)' * P * (x - v) (with P > 0).

    % Calculate the Lyapunov threshold value
    % Args:
    % v: current applied reference
    % d: part of the constraint
    % c: part of the constraint
    % P: used for calculating Lyapunov value

    gamma_value = (-c' * v + d)^2 / (c' * inv(P) * c);

end