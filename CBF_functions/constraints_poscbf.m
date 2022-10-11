function [obj] = constraints_poscbf(A,B,obs_radius)
% High order control barrier function
syms p_x p_y p_z v_x v_y v_z obs_p_x obs_p_y obs_p_z
x = [p_x;p_y;p_z;v_x;v_y;v_z];
obs_pos = [obs_p_x;obs_p_y;obs_p_z];
f = A * x;
g = B;
% cbf = (x(1) - obs.pos(obs_det,1))^2 + (x(3) - obs.pos(obs_det,2))^2 + (x(5) - obs.pos(obs_det,3))^2 - obs_radius^2;
cbf = (x(1) - obs_pos(1))^2 + (x(3) - obs_pos(2))^2 + (x(5) - obs_pos(3))^2 - obs_radius^2;
obj.xdim = size(x, 1);
obj.udim = size(g, 2); 
dcbf = simplify(jacobian(cbf, x));
lf_cbf = dcbf * f;
lg_cbf = dcbf * g;
ddcbf = simplify(jacobian(lf_cbf, x));
lflf_cbf = ddcbf * f;
lglf_cbf = ddcbf * g;
remain_cbf = lflf_cbf + 40*lf_cbf + 400*cbf;
obj.cbf = matlabFunction(cbf,'vars',{x,obs_pos});
obj.lglf_cbf = matlabFunction(lglf_cbf, 'vars', {x,obs_pos});
obj.remain_cbf = matlabFunction(remain_cbf, 'vars', {x,obs_pos});
end