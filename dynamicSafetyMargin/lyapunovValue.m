function val = lyapunovValue(x, v, P)
    % Calculates the Lyapunov value.
    % Args:
    %     x: current state
    %     v: current applied reference
    %     P: matrix used for calculation

    val = (x - v)' * P * (x - v);
end