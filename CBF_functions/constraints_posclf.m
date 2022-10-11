function [Con_,obj] = constraints_posclf(m,k1,eplison1,eta1,A,B)
% Time varying control lyapunov function
syms p_x p_y p_z v_x v_y v_z u_x u_y u_z pd_x pd_y pd_z vd_x vd_y vd_z ud_x ud_y ud_z
x = [p_x;v_x;p_y;v_y;p_z;v_z];
u = [u_x;u_y;u_z];
ref_x = [pd_x;vd_x;pd_y;vd_y;pd_z;vd_z];
ref_u = [ud_x;ud_y;ud_z];
f = A * x;
g = B;
error_pos = [x(1)-ref_x(1);x(3)-ref_x(3);x(5)-ref_x(5)];
error_vel = [x(2)-ref_x(2);x(4)-ref_x(4);x(6)-ref_x(6)];
clf = (1/2)*m*(error_vel.')*(error_vel) + (1/2)*k1*(error_pos.')*(error_pos) + eplison1*(error_pos.')*(error_vel);
dclf = simplify(jacobian(clf,x));
lf_clf = dclf * f;
lg_clf = dclf * g;
dot_clf_x = lf_clf + lg_clf * u;
% dot_clf_t = (-ud_x*(p_x+v_x-vd_x-pd_x)) + (-vd_x * (p_x+v_x-pd_x-vd_x)) + (-ud_y*(p_y+v_y-pd_y-vd_y)) + (-vd_y*(p_y+v_y-pd_y-vd_y));
dot_clf_t = (p_x+v_x-pd_x-vd_x) * (-ud_x - vd_x) + (p_y+v_y-pd_y-vd_y) * (-ud_y - vd_y) + (p_z+v_z-pd_z-vd_z) * (-ud_z - vd_z);

% Setting state and input dimension.
obj.xdim = size(x, 1);
obj.udim = size(g, 2);
obj.lf_clf = matlabFunction(lf_clf,'vars',{x,ref_x});
obj.lg_clf = matlabFunction(lg_clf,'vars',{x,ref_x});
obj.dot_clf_t = matlabFunction(dot_clf_t,'vars',{x,ref_x,ref_u});
obj.clf = matlabFunction(clf,'vars',{x,ref_x});
% Build Constraints
Con_ = dot_clf_t + dot_clf_x + eta1*clf;

end