% 
% function flag  = agentObstaclesCollisionCheck( ...
%     agent_position, ...
%     agent_size, ...
%     obstacle_position, ...
%     obstacle_sizes, ...
%     safety_margin_universal)
% 
%     flag = 0;
% 
%     for j = 1:size(obstacle_position, 2)
% 
%         dist = norm(agent_position - obstacle_position(:,j)) ...
%             - agent_size - obstacle_sizes(j) - safety_margin_universal;
% 
%         if dist < 0
%             disp("collision detected")
%             disp(agent_position);
%             disp(obstacle_position(:,j));
%             flag = 1;
%             return;
%         end
% 
%     end
% 
% end