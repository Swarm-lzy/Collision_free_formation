function [index_obs,dis_obs,flag_obs] = obstacleDetect(x,obs,obsrange)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
dis_obs = zeros(obs.num,1);
for k = 1:obs.num
    dis_obs(k) = sqrt((x(1)-obs.pos(k,1))^2 + (x(2)-obs.pos(k,2))^2 + (x(3) - obs.pos(k,3))^2);
end
% index_obs = find(dis_obs == min(dis_obs)); % Find the obstacle with minimal distance
index_obs = find(dis_obs <= obsrange); % Find the obstacles in detection range
if ~isempty(index_obs)
    flag_obs = 1;
else
    flag_obs = 0;
end
end