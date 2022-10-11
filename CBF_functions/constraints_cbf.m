function [obj] = constraints_cbf(param, CBFparam)
%% Modified CBF for quadrotor
% dot(R*e3,R*e3) = 1
syms p_x p_y p_z v_x v_y v_z R11 R12 R13 R21 R22 R23 R31 R32 R33 W1 W2 W3 pd_x pd_y pd_z vd_x vd_y vd_z
R = [R11,R12,R13; R21,R22,R23; R31,R32,R33];
W = [W1;W2;W3];
p = [p_x;p_y;p_z];
v = [v_x;v_y;v_z];
pd = [pd_x;pd_y;pd_z];
% vd = zeros(dim,1);
vd = [vd_x;vd_y;vd_z];
% ad = zeros(dim,1);
radius_dot = 0;
radius_2dot = 0;
e3_hat = hat_map(param.e3);
W_hat = hat_map(W);

r = p - pd;
q = R*param.e3;
s = dot(r,q);
s_dot = dot((v-vd),(R*param.e3)) + dot((p-pd),(R*cross(W,param.e3)));
[sigma,sigma_dot,sigma_2dot] = sigmaConstruct(CBFparam.a1,CBFparam.a2,CBFparam.a3,s);

g = dot(r,r) - CBFparam.beta*((CBFparam.safeDis)^2) - sigma;
g_dot = 2 * dot(v-vd,p-pd) - CBFparam.beta*radius_dot - sigma_dot*s_dot;


h = CBFparam.gamma*g + g_dot;
h_dot_1 = CBFparam.gamma*g_dot + 2*dot((v-vd),(v-vd)) - CBFparam.beta*radius_2dot - sigma_2dot*(s_dot^2);
h_dot_2 = sigma_dot*( 2*dot((v-vd),(R*cross(W,param.e3))) + (p-pd)'*R*W_hat^2*param.e3 );
h_dot_3_f = 2*dot(r,param.g*param.e3) - sigma_dot*(dot( (R*param.e3), (param.g*param.e3) )) ;    % 考虑ge3与W x JW
h_dot_3_M = - sigma_dot*(  (r'*R*e3_hat) * (param.J\(cross(W,param.J*W))) ); 

TAU_3 = h_dot_1 + h_dot_2 + h_dot_3_f + h_dot_3_M; % 1*1
TAU_1 = -(2*s - sigma_dot)/param.m;           % 1*1
TAU_2 = (sigma_dot*r'*R*e3_hat) * inv(param.J); % 1*3
remaining = CBFparam.gamma*h^3 + TAU_3; % 1*1
obj.TAU_1_func = matlabFunction(TAU_1,'vars',{p,v,R,W,pd,vd});
obj.TAU_2_func = matlabFunction(TAU_2,'vars',{p,v,R,W,pd,vd});
obj.remaining_func = matlabFunction(remaining,'vars',{p,v,R,W,pd,vd});


thd = 2*s - sigma_dot;
obj.thd_func = matlabFunction(thd,'vars',{p,v,R,W,pd,vd});
end

function [sigma,sigma_dot,sigma_2dot] = sigmaConstruct(a1,a2,a3,s)
    sigma = -a1*atan(a2*s+a3);
    sigma_dot = -(a1*a2)/((a3 + a2*s)^2 + 1);
    sigma_2dot = (2*a1*a2^2*(a3 + a2*s))/((a3 + a2*s)^2 + 1)^2;

end