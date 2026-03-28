function ref = getRefFromPath(s, path)
    lower_idx = floor(s);
    upper_idx = ceil(s);
    vec_delta = path(upper_idx,:) - path(lower_idx,:);
    ref = (path(lower_idx,:) + (s - lower_idx) * vec_delta)';
end