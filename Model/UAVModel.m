function [dX] = UAVModel(param,t,X,F)
dX = zeros(18, 1);
R_now = reshape(X(7:15), 3, 3);
W_now = X(16:18);
f = F(1);
M = F(2:4);

dx = X(4:6);
dv = param.g*param.e3 - (f/param.m)*R_now*param.e3;
dR = R_now*hat_map(W_now);
dW = param.J\(-cross(W_now, param.J*W_now) + M);


dX(1:3) = dx;
dX(4:6) = dv;
dX(7:15) = reshape(dR, 9, 1);
dX(16:18) = dW;
end