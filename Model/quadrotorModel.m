function [dX] = quadrotorModel(v,R,W,f,M,param,t)
% dX = zeros(18,1);
% dX(1:3) = X(4:6);
% dX(4:6) = param.g*param.e3 - f*(reshape(X(7:15),3,3))*param.e3/param.m;
% dX(7:15) = reshape(reshape(X(7:15),3,3)*hat_map(X(16:18)),[],1);
% dX(16:18) = param.J\(-cross(X(16:18), param.J*X(16:18)) + M);

% position control
x_dot = v;
v_dot = param.g*param.e3 - f*R*param.e3/param.m;
% attitude control
R_dot = R*hat_map(W);
W_dot = param.J\(-cross(W, param.J*W) + M);

dX = [x_dot; v_dot; reshape(R_dot,9,1); W_dot];
end