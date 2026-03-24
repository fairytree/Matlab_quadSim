% Get v position (x, y, z) by index from path
function result = getVPosition(s, path)
   
    lower_idx = floor(s);
    upper_idx = ceil(s);

    vector_delta = path(upper_idx, :) - path(lower_idx, :);

    result_xyz_row = path(lower_idx,:) + (s - lower_idx) * vector_delta;

    % reshape to column vector and append desired yaw angle
    result_xyz_column = reshape(result_xyz_row, [], 1);
    result =[result_xyz_column; 0.0];
    
end