
function collided = lineSegmentAndSphereCollision(segment_start, ...
    segment_direction_normalized, segment_length, sphere_radius, ...
    sphere_center)
    % check if the given line segment and sphere is colliding

    % see https://education.siggraph.org/static/HyperGraph/raytrace/rtinter1.htm
    b = 2 * (segment_direction_normalized(1) * (segment_start(1) - sphere_center(1)) ...
        + segment_direction_normalized(2) * (segment_start(2) - sphere_center(2)) ...
        + segment_direction_normalized(3) * (segment_start(3) - sphere_center(3)));
    c = (segment_start(1) - sphere_center(1))^2 + ...
        (segment_start(2) - sphere_center(2))^2 + ...
        (segment_start(3) - sphere_center(3))^2 - sphere_radius^2;

    discriminant = b^2 - 4 * c;

    % see if discriminant is negative or not
    if discriminant < 0
        collided = false;
    else
        t_0 = (-b + sqrt(discriminant)) / 2;
        t_1 = (-b - sqrt(discriminant)) / 2;

        % see if the line collided with the sphere on the line segment
        if (0 <= t_0 && t_0 <= segment_length) || (0 <= t_1 && t_1 <= segment_length)
            collided = true;
        else
            collided = false;
        end
    end
end