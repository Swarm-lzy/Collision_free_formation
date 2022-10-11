function [dX] = pointModel(t,X,F)
dX = zeros(6,1);

dx = X(4:6);
dv = F;

dX(1:3) = dx;
dX(4:6) = dv;
end