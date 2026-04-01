% Compute cumulative arc length from the start for a sequence of (x, y, z) positions.
% positions: N x 3 matrix of [x, y, z] positions
% Returns: 1 x N vector of cumulative distances from the first position.
function length_from_start = getLengthFromStartXYZ(positions)
    num_points = size(positions, 1);
    length_from_start = zeros(1, num_points);
    for idx = 2:num_points
        length_from_start(idx) = length_from_start(idx - 1) + ...
            norm(positions(idx, :) - positions(idx - 1, :));
    end
end
