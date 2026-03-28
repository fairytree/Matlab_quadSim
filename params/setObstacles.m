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
rect_obs = [2, 0, 4, 4; 0, 1, 1, 2];
buffer = 0.3;