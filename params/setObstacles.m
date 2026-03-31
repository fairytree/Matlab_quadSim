% format for the list of rectangular prisms:
% each prism is represented by a pair of vertices--the one with the lowest x, y, z coordinates and the one with the highest
% call these points (x1, y1, z1) and (x2, y2, z2) respectively
% the list looks like this:
% [
%     xa1, ya1, za1, xa2, ya2, za2; % obstacle A
%     xb1, yb1, zb1, xb2, yb2, zb2; % obstacle B
%     ...
% ]
% in the case of a unicycle, it's 2D, so there's no z-coordinate
rect_obs = [
    3.0, 1.0, 7.0, 3.0;   % original large wall
    -0.5, 1.0, 1.0, 2.0;   % original small block
    2.2, 2.5, 3.7, 3.5;   % mid-left block
    4.0, 4.0, 5.5, 5.0;   % upper-right block
    0.5, 4.5, 2.5, 5.5;   % upper-left block
];
buffer = 0.15; % MPC safety margin
rrt_safety_margin = 0.3;