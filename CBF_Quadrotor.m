close all
clear
clc
% 添加路径
addpath('utils\');
addpath('Param\');
addpath('CBF_functions\');
addpath('Control\');
addpath('Model\');
addpath('Plot_functions\');
addpath('quadrotor_model\');
[T,obs,MA,param,obsCBF,collCBF] = Paramter();
%% Initial state
% position level
pos_Init = [0;0;0];
vel_Init = [0;0;0];
% orientation level
R_Init = eye(3);  % 旋转矩阵
W_Init = [0;0;0]; % 角速度
%% Varibale set for position level
% actual
pos_valts = [pos_Init,NaN(param.dim,T.tSteps)];
vel_valts = [vel_Init, NaN(param.dim,T.tSteps)];
% reference
ref_pos_valts = NaN(param.dim,T.tSteps);
ref_vel_valts = NaN(param.dim,T.tSteps);
%% Variable set for orientation level
% actual
R_valts = [reshape(R_Init,[],1),NaN(param.dim*3,T.tSteps)];
W_valts = [reshape(W_Init,[],1),NaN(param.dim,T.tSteps)];
% reference
% b1d_valts = NaN(3,tSteps);
% b1d_dot_valts = NaN(3,tSteps);
% b1d_2dot_valts = NaN(3,tSteps);
%% Build control barrier function
% [Con_clf,obj_clf] = constraints_posclf(para.m,para.k1,para.epsilon1,para.eta1,A,B);
% [obj_cbf] = constraints_poscbf(A,B,Obs);
[obj] = constraints_cbf(param, obsCBF);
Thdvalue = [];
hwait=waitbar(0,'½øÐÐÖÐ>>>>>>>>>>');
%% Simulation Loop
for step = 1:T.tSteps
    t = T.taxis(step);
    tnext = T.taxis(step+1);
    % 获取reference的状态
    [ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap, Bd] = reference(t);
    Xd = [ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap];
    ref_pos_valts(:,step) = ref_pos;
    % 获取当前时刻的状态
    x = pos_valts(:,step);
    v = vel_valts(:,step);
    R = reshape(R_valts(:,step), 3, 3);
    W = W_valts(:,step);
    X_now = [x;v;R_valts(:,step);W];
    % 计算名义的f与名义的M
    f = posCtrl(param.kp, param.kv, param.m, param.g, param.e3, R, x, v, Xd);
    [~,~,~,M] = mntCtrl(param, X_now, Xd, Bd);
    control = [f;M];
    control_nom = control;
    %% CBF
    % 判断在探测范围内的障碍物
    [index_obs,dis_obs,flag_obs] = obstacleDetect(x,obs,obsCBF.range);
    if flag_obs == 1
        Con_A = [];
        Con_b = [];
        for k = 1:length(index_obs)
            obs_ind = obs.pos(index_obs(k),:)';
            obs_vel = zeros(param.dim,1);
            TAU_1 = obj.TAU_1_func(x,v,R,W,obs_ind,obs_vel);
            TAU_2 = obj.TAU_2_func(x,v,R,W,obs_ind,obs_vel);
            remaining = obj.remaining_func(x,v,R,W,obs_ind,obs_vel);
            thdvalue = obj.thd_func(x,v,R,W,obs_ind,obs_vel);
            Thdvalue = [Thdvalue,thdvalue];
            Con_A_temp = -[TAU_1,TAU_2];
            Con_b_temp = remaining;
            %组合
            Con_A = [Con_A;Con_A_temp];
            Con_b = [Con_b;Con_b_temp];
        end
        options =  optimset('Display','notify');
        H = eye(4);
        f_ = -eye(4) * control_nom;
        [control, ~, exitflag, ~] = quadprog(H, f_, Con_A, Con_b, [], [], [], [], [], options);
    end
    % 传递f与M到四旋翼模型    
    [~, X_new] = ode45(@(t, x) UAVModel(param, t, x, control), [0, T.tUnitStep], X_now, control);
    % update
    pos_valts(:,step+1) = X_new(end,1:3);
    vel_valts(:,step+1) = X_new(end,4:6);
    R_valts(:,step+1) = X_new(end,7:15);
    W_valts(:,step+1) = X_new(end,16:18);
    waitbar(step/T.tSteps,hwait,'simulating...');
end
close(hwait);
%% Diagram
% error_1 = pos_valts(:,1:tSteps) - ref_pos_valts;
% error_2 = vel_valts(:,1:tSteps) - ref_vel_valts;

figure(1)
% min_temp = min(min(pos_valts(1,:)), min(pos_valts(2,:)));
% lim_min = min(min_temp, min(pos_valts(3,:)));
% max_temp = max(max(pos_valts(1,:)), max(pos_valts(2,:)));
% lim_max = max(max_temp, max(pos_valts(2,:)));
for k = 1:obs.num
    obs_h = obstaclePlot(obs.pos(k,:),obs.rad); 
%     scatter3(obs.pos(k,1),obs.pos(k,2),obs.pos(k,3));
    hold on
end
plot3(ref_pos_valts(1,:),ref_pos_valts(2,:),ref_pos_valts(3,:),'r','LineWidth',1); hold on %参考轨迹用红色
plot3(pos_valts(1,:),pos_valts(2,:),pos_valts(3,:), 'k','LineWidth',1); %真实的轨迹用黑色
max_l = max(ceil(abs(pos_valts(:))), [], 'all');
xlim([-max_l, max_l]);
ylim([-max_l, max_l]);
zlim([-max_l, max_l]);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
%% 动态的运动轨迹
