function lyap_val = lyapVal(state, ref_state)
    diff = state - ref_state;
    lyap_val = diff' * diff;
end