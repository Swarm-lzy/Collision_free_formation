function [dX] = DynamicsCBF(t, X, param, obs, obj_obs)
% Split the states
x = X(1:3);
v = X(4:6);
R = reshape(X(7:15), 3, 3);
W = X(16:18);

% Reference trajectory
[ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap, ref_Bd] = reference(t);
Xd = [ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap];
% Position control
f = posCtrl(param.kp, param.kv, param.m, param.g, param.e3, R,...
            x, v, Xd);
% Attitude control
[~, ~, ~, M] = mntCtrl(param, X, Xd, ref_Bd);
u_nom = [f;M];


[index_obs,~,flag_obs] = judgeCollision(x,obs,param.range);
if flag_obs == 1
%     Con_A = [eye(3);-eye(3)];
%     Con_b = [ones(3,1)*param.u_max;ones(3,1)*param.u_min];
    Con_A = [];
    Con_b = [];
    for k = 1:length(index_obs)
        % obtain the obstacle position in range
        obs_pos = obs.pos(index_obs(k),:)';
        TAU_1 = obj_obs.TAU_1_func(x,v,R,W,obs_pos);
        TAU_2 = obj_obs.TAU_2_func(x,v,R,W,obs_pos);
        remaining = obj_obs.remaining_func(x,v,R,W,obs_pos);
        Con_A = -[TAU_1,TAU_2];
        Con_b = remaining;
        Con_A = [Con_A;Con_A];
        Con_b = [Con_b;Con_b];
    end
    options =  optimset('Display','notify');
    H = eye(4);
    f_ = -eye(4) * u_nom;
    [u, ~, ~, ~] = quadprog(H, f_, Con_A, Con_b, [], [], [], [], [], options);
    f = u(1);
    M = u(2:4);
end

% Equations of motion for UAV
% x_dot = v;
% v_dot = param.g*param.e3 - f*R*param.e3/param.m;
% R_dot = R*hat(W);
% W_dot = param.J\(-cross(W, param.J*W) + M);
% 
% dX = [x_dot; v_dot; reshape(R_dot, 9, 1); W_dot];
dX = quadrotorModel(v,R,W,f,M,param);
end