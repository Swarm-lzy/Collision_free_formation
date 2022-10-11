function [index_obs,dis_obs,flag_obs,index_agent,dis_agent,flag_coll] = judgeCollision(i,x,obs,agent,N,obsrange,collrange)
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

dis_agent = zeros(N,1);
for j = 1:N 
    dis_agent(j) = sqrt((x(1) - agent(3*j-2))^2 + (x(2) - agent(3*j-1))^2 + (x(3) - agent(3))^2);
end
% index_agent = find(dis_agent == min(dis_agent)); % Find the agent with minimal distance
index_agent = find(dis_agent <= collrange);
index_agent(index_agent==i) = [];
if ~isempty(index_agent)
    flag_coll = 1;
else
    flag_coll = 0;
end    
end

