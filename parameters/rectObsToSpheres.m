function [centers, radii] = rectObsToSpheres(rect_obs, max_sphere_radius)
% rectObsToSpheres  Decompose box obstacles into inscribed spheres.
%
%   [centers, radii] = rectObsToSpheres(rect_obs, max_sphere_radius)
%
%   INPUTS
%     rect_obs           [N_r × 6]  each row = [x_lo y_lo z_lo x_hi y_hi z_hi]
%     max_sphere_radius  scalar     upper bound on sphere radius
%
%   OUTPUTS
%     centers  [M × 3]  sphere centre positions  (append to obstacles)
%     radii    [M × 1]  sphere radii             (append to obstacle_sizes)
%
%   Spheres are as large as possible but never extend outside the box.
%   The radius starts as min(max_sphere_radius, half the smallest box
%   dimension).  Along each axis we use ceil(extent / (2*r)) spheres so
%   the entire box is covered, then shrink the radius to the largest
%   value that still keeps every sphere fully inside the box.
%
%   Example: a 1×2×2 box → r = 0.5, n = [1 2 2], 4 spheres of radius 0.5.

    centers = [];
    radii   = [];

    for k = 1:size(rect_obs, 1)
        lo = rect_obs(k, 1:3)';
        hi = rect_obs(k, 4:6)';
        extent = hi - lo;                          % [dx; dy; dz]

        % Initial radius = min(max_sphere_radius, half of smallest side)
        r0 = min(max_sphere_radius, min(extent) / 2);

        % Number of spheres per axis – use ceil so no gaps remain
        n = max(1, ceil(extent / (2 * r0)));

        % Shrink radius so that n spheres of diameter 2*r exactly span
        % each axis:  n(i)*2*r <= extent(i)  →  r <= extent(i)/(2*n(i))
        % Take the tightest (minimum) across all axes.
        r = min(extent ./ (2 * n));

        % Spacing between adjacent sphere centres along each axis.
        % When n(i)==1, the single centre sits at the box midpoint.
        spacing = zeros(3,1);
        for d = 1:3
            if n(d) == 1
                spacing(d) = 0;
            else
                spacing(d) = (extent(d) - 2*r) / (n(d) - 1);
            end
        end

        % Place sphere centres on the grid
        for ix = 1:n(1)
            cx = lo(1) + r + (ix - 1) * spacing(1);
            for iy = 1:n(2)
                cy = lo(2) + r + (iy - 1) * spacing(2);
                for iz = 1:n(3)
                    cz = lo(3) + r + (iz - 1) * spacing(3);
                    centers = [centers; cx, cy, cz]; %#ok<AGROW>
                    radii   = [radii;   r];          %#ok<AGROW>
                end
            end
        end
    end
end
