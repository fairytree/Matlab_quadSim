function val = lyapunovValue(x, v, P)
    val = (x - v)' * P * (x - v);
end

function lyapunov_value = lyapunovValue(state, ref)
    diff = state - ref;
    lyapunov_value = diff' * diff;
end