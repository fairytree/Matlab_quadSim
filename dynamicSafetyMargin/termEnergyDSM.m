function term_energy_DSM = termEnergyDSM(state, ref, params)
    ref_state = ref;
    lyap_val = lyapVal(state, ref_state);
    speed_lyap_thresh = speedLyapThresh(ref_state, params);
    obs_lyap_thresh = obsLyapThresh(ref_state, params);
    term_energy_DSM = min([speed_lyap_thresh, obs_lyap_thresh]) - lyap_val;
end