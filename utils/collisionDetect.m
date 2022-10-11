function [index_coll,dis_coll,flag_coll] = collisionDetect(i,pos,all_UAV,range)
N = length(all_UAV)/3;
dis_coll = zeros(1,N);
flag_coll = zeros(1,N);
for j = 1:N 
    dis_coll(j) = sqrt((pos(1) - all_UAV(3*j-2))^2 + (pos(2) - all_UAV(3*j-1))^2 + (pos(3) - all_UAV(3))^2);
end
% index_agent = find(dis_agent == min(dis_agent)); % Find the agent with minimal distance
index_coll = find(dis_coll <= range);
index_coll(index_coll==i) = [];

for i = 1:length(index_coll)
    ind = index_coll(i);
    flag_coll(ind) = 1;
end   


end