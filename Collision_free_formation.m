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
% 生成参数
[T,obs,MA,param,obsCBF,collCBF] = Paramter();
%% 初始化
% 实际状态
act.xf_pos_Init = [3,  -3,  0,  0
                   0,   0, -3,  3
                   0,   0,  0,  0];
act.xf_vel_Init = zeros(param.dim,MA.N_F);
act.xf_acc_Init = zeros(param.dim,MA.N_F);
act.xf_jerk_Init = zeros(param.dim,MA.N_F);
act.xf_snap_Init = zeros(param.dim,MA.N_F);
act.xf_R_Init = [eye(3),eye(3),eye(3),eye(3)];
act.xf_W_Init = zeros(param.dim,MA.N_F);
% 估计状态
rng(123)
est.xf_pos_Init = 2* rand(param.dim,MA.N_F) * 3 -  3;
est.xf_vel_Init = rand(param.dim,MA.N_F) * 2;
est.xf_acc_Init = rand(param.dim,MA.N_F) * 2;
est.xf_jerk_Init = rand(param.dim,MA.N_F) * 2;
est.xf_snap_Init = rand(param.dim,MA.N_F) * 2;
%% 轨迹变量集合
act.xl_pos_valts = NaN(MA.N_L*param.dim,T.tSteps);
act.xl_vel_valts = NaN(MA.N_L*param.dim,T.tSteps);
act.xl_acc_valts = NaN(MA.N_L*param.dim,T.tSteps);
act.xl_jerk_valts = NaN(MA.N_L*param.dim,T.tSteps);
act.xl_snap_valts = NaN(MA.N_L*param.dim,T.tSteps);
act.xf_pos_valts = [reshape(act.xf_pos_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act.xf_vel_valts = [reshape(act.xf_vel_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act.xf_acc_valts = [reshape(act.xf_acc_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act.xf_jerk_valts = [reshape(act.xf_jerk_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act.xf_snap_valts = [reshape(act.xf_snap_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act.xf_R_valts = [reshape(act.xf_R_Init,[],1), NaN(MA.N_F*param.dim*param.dim, T.tSteps)];
act.xf_W_valts = [reshape(act.xf_W_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps);];

des.xf_pos_valts = NaN(MA.N_F*param.dim,T.tSteps);
des.xf_vel_valts = NaN(MA.N_F*param.dim,T.tSteps);
des.xf_acc_valts = NaN(MA.N_F*param.dim,T.tSteps);
des.xf_jerk_valts = NaN(MA.N_F*param.dim,T.tSteps);
des.xf_snap_valts = NaN(MA.N_F*param.dim,T.tSteps);

est.xf_pos_valts = [reshape(est.xf_pos_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
est.xf_vel_valts = [reshape(est.xf_vel_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
est.xf_acc_valts = [reshape(est.xf_acc_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
est.xf_jerk_valts = [reshape(est.xf_jerk_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
est.xf_snap_valts = [reshape(est.xf_snap_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
est.xf_pos_valts = [reshape(est.xf_pos_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
est.xf_vel_valts = [reshape(est.xf_vel_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
est.xf_acc_valts = [reshape(est.xf_acc_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
est.xf_jerk_valts = [reshape(est.xf_jerk_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
est.xf_snap_valts = [reshape(est.xf_snap_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];

[obj_obs] = constraints_cbf(param, obsCBF);
[obj_coll] = constraints_cbf(param, collCBF);
hwait=waitbar(0,'½øÐÐÖÐ>>>>>>>>>>');
for step = 1:T.tSteps
    tnow = T.taxis(step);
    tnext = T.taxis(step+1);
    [pos_l,vel_l,acc_l,jerk_l,snap_l, Bd] = leader_trajectory(tnow);
    Xd_l = [pos_l,vel_l,acc_l,jerk_l,snap_l];
    act.xl_pos_valts(:,step) = pos_l;
    act.xl_vel_valts(:,step) = vel_l;
    act.xl_acc_valts(:,step) = acc_l;
    act.xl_jerk_valts(:,step) = jerk_l;
    act.xl_snap_valts(:,step) = snap_l;
    for i = 1:MA.N_F
        des.xf_pos_valts(3*i-2:3*i,step) = pos_l + MA.formation_shape(:,i+1);
        des.xf_vel_valts(3*i-2:3*i,step) = vel_l;
        des.xf_acc_valts(3*i-2:3*i,step) = acc_l;
        des.xf_jerk_valts(3*i-2:3*i,step) = jerk_l;
        des.xf_snap_valts(3*i-2:3*i,step) = snap_l;
    end
    %% Distributed trajectory generation 
    for i = 1:MA.N_F
        error_vel = zeros(param.dim,1);
        error_pos = zeros(param.dim,1);
        error_acc = zeros(param.dim,1);
        error_jerk = zeros(param.dim,1);
        error_snap = zeros(param.dim,1);
        % snap
        for j = 1:MA.N_F
            error_snap = error_snap + MA.W(i+1,j+1)*(est.xf_snap_valts(3*i-2:3*i,step) - est.xf_snap_valts(3*j-2:3*j,step));
        end
        error_snap = error_snap + MA.W(i+1,1)*(est.xf_snap_valts(3*i-2:3*i,step) - act.xl_snap_valts(:,step));
        dot_snap = - param.alpha_1 * error_snap - param.alpha_2 * sign(error_snap) - param.alpha_3 * diag((abs(error_snap))) * sign(error_snap);
        est.xf_snap_valts(3*i-2:3*i,step+1) = est.xf_snap_valts(3*i-2:3*i,step) + dot_snap * T.tUnitStep;
        % jerk
        for j = 1:MA.N_F
            error_jerk = error_jerk + MA.W(i+1,j+1)*(est.xf_jerk_valts(3*i-2:3*i,step) - est.xf_jerk_valts(3*j-2:3*j,step));
        end
        error_jerk = error_jerk + MA.W(i+1,1)*(est.xf_jerk_valts(3*i-2:3*i,step) - act.xl_jerk_valts(:,step));
        dot_jerk = - param.alpha_1 * error_jerk - param.alpha_2 * sign(error_jerk) - param.alpha_3 * diag((abs(error_jerk))) * sign(error_jerk);
        est.xf_jerk_valts(3*i-2:3*i,step+1) = est.xf_jerk_valts(3*i-2:3*i,step) + dot_jerk * T.tUnitStep;
        % acceleration
        for j = 1:MA.N_F
            error_acc = error_acc + MA.W(i+1,j+1)*(est.xf_acc_valts(3*i-2:3*i,step) - est.xf_acc_valts(3*j-2:3*j,step));
        end
        error_acc = error_acc + MA.W(i+1,1)*(est.xf_acc_valts(3*i-2:3*i,step) - act.xl_acc_valts(:,step));
        dot_acc = - param.alpha_1 * error_acc - param.alpha_2 * sign(error_acc) - param.alpha_3 * diag((abs(error_acc))) * sign(error_acc);
        est.xf_acc_valts(3*i-2:3*i,step+1) = est.xf_acc_valts(3*i-2:3*i,step) + dot_acc * T.tUnitStep;
        % velocity
        for j = 1:MA.N_F
            error_vel = error_vel + MA.W(i+1,j+1)*(est.xf_vel_valts(3*i-2:3*i,step) - est.xf_vel_valts(3*j-2:3*j,step));
        end
        error_vel = error_vel + MA.W(i+1,1)*(est.xf_vel_valts(3*i-2:3*i,step) - act.xl_vel_valts(:,step));
        dot_vel = - param.alpha_1 * error_vel - param.alpha_2 * sign(error_vel) - param.alpha_3 * diag((abs(error_vel))) * sign(error_vel);
        est.xf_vel_valts(3*i-2:3*i,step+1) = est.xf_vel_valts(3*i-2:3*i,step) + dot_vel * T.tUnitStep;
        % position
        for j = 1:MA.N_F
            error_pos = error_pos + MA.W(i+1,j+1)*(est.xf_pos_valts(3*i-2:3*i,step) - est.xf_pos_valts(3*j-2:3*j,step) - (MA.formation_shape(:,i+1) - MA.formation_shape(:,j+1)));
        end
        error_pos = error_pos + MA.W(i+1,1)*(est.xf_pos_valts(3*i-2:3*i,step) - act.xl_pos_valts(:,step) - (MA.formation_shape(:,i+1) - MA.formation_shape(:,1)));
        dot_pos = est.xf_vel_valts(3*i-2:3*i,step) - param.beta_1 * error_pos - param.beta_2 * sign(error_pos);
        est.xf_pos_valts(3*i-2:3*i,step+1) = est.xf_pos_valts(3*i-2:3*i,step) + dot_pos * T.tUnitStep;
    end
    %% Trajectory tracking with collision avoidance
    for i = 1:MA.N_F
        ref_pos = est.xf_pos_valts(3*i-2:3*i,step);
        ref_vel = est.xf_vel_valts(3*i-2:3*i,step);
        ref_acc = est.xf_acc_valts(3*i-2:3*i,step);
        ref_jerk = est.xf_jerk_valts(3*i-2:3*i,step);
        ref_snap = est.xf_snap_valts(3*i-2:3*i,step);
        Xd_f = [ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap];

        act_pos = act.xf_pos_valts(3*i-2:3*i,step);
        act_vel = act.xf_vel_valts(3*i-2:3*i,step);
        act_R_temp = act.xf_R_valts(9*i-8:9*i,step);
        act_R = reshape(act_R_temp,3,3);
        act_W = act.xf_W_valts(3*i-2:3*i,step);
        act_state = [act_pos;act_vel;act_R_temp;act_W];
        f = posCtrl(param.kp, param.kv, param.m, param.g, param.e3, act_R, act_pos, act_vel, Xd_f);
        [~,~,~,M] = mntCtrl(param, act_state, Xd_f, Bd);
        control = [f;M];
        control_nom = control;
        %% 避障模块
        % 检测碰撞
        [index_obs,dis_obs,flag_obs,index_agent,dis_agent,flag_coll] = judgeCollision(i, act_pos, obs, act.xf_pos_valts(:,step), MA.N_F, obsCBF.range, collCBF.range);
        if flag_obs == 1 || flag_coll == 1
            Con_A = [];
            Con_b = [];
            if flag_obs == 1 
                for k = 1:length(index_obs)
                    obs_pos = obs.pos(index_obs(k),:)';
                    obs_vel = zeros(param.dim,1);
                    TAU_1_obs = obj_obs.TAU_1_func(act_pos,act_vel,act_R,act_W,obs_pos,obs_vel);
                    TAU_2_obs = obj_obs.TAU_2_func(act_pos,act_vel,act_R,act_W,obs_pos,obs_vel);
                    remaining_obs = obj_obs.remaining_func(act_pos,act_vel,act_R,act_W,obs_pos,obs_vel);
                    Con_A_obs = -[TAU_1_obs,TAU_2_obs];
                    Con_b_obs = remaining_obs;
                    %组合
                    Con_A = [Con_A;Con_A_obs];
                    Con_b = [Con_b;Con_b_obs];
                end
            end
            if flag_coll == 1
%                 Con_A = [];
%                 Con_b = [];
                for k = 1:length(index_agent)
                    agent_index = index_agent(k);
                    agent_pos = act.xf_pos_valts(3*agent_index-2:3*agent_index,step);
                    agent_vel = act.xf_vel_valts(3*agent_index-2:3*agent_index,step);
                    TAU_1_coll = obj_coll.TAU_1_func(act_pos,act_vel,act_R,act_W,agent_pos,agent_vel);
                    TAU_2_coll = obj_coll.TAU_2_func(act_pos,act_vel,act_R,act_W,agent_pos,agent_vel);
                    remaining_coll = obj_coll.remaining_func(act_pos,act_vel,act_R,act_W,agent_pos,agent_vel);
                    Con_A_coll = -[TAU_1_coll,TAU_2_coll];
                    Con_b_coll = remaining_coll;
                 end
                 Con_A = [Con_A;Con_A_coll];
                 Con_b = [Con_b;Con_b_coll];
             end 
            options =  optimset('Display','notify');
            H = eye(4);
            f_ = -eye(4) * control_nom;
            [control, ~, exitflag, ~] = quadprog(H, f_, Con_A, Con_b, [], [], [], [], [], options);
        end
        [~, X_new] = ode45(@(t, x) UAVModel(param, t, x, control), [0, T.tUnitStep], act_state, control);
        act.xf_pos_valts(3*i-2:3*i,step+1) = X_new(end,1:3);
        act.xf_vel_valts(3*i-2:3*i,step+1) = X_new(end,4:6);
        act.xf_R_valts(9*i-8:9*i,step+1) = X_new(end,7:15);
        act.xf_W_valts(3*i-2:3*i,step+1) = X_new(end,16:18);
    end
    waitbar(step/T.tSteps,hwait,'simulating...');
end
close(hwait);
% 绘制轨迹图
figure(1)
plot3(act.xf_pos_valts(1,:), act.xf_pos_valts(2,:), act.xf_pos_valts(3,:),'b','LineWidth',1);
hold on
plot3(act.xf_pos_valts(4,:),act.xf_pos_valts(5,:),act.xf_pos_valts(6,:),'r','LineWidth',1);
hold on
plot3(act.xf_pos_valts(7,:),act.xf_pos_valts(8,:),act.xf_pos_valts(9,:),'k','LineWidth',1);
hold on
plot3(act.xf_pos_valts(10,:),act.xf_pos_valts(11,:),act.xf_pos_valts(12,:),'m','LineWidth',1);
hold on
for k = 1:obs.num
    obs_h = obstaclePlot(obs.pos(k,:),obs.rad); 
    hold on
end
x_max = max(ceil(abs(act.xl_pos_valts(:))), [], 'all');
xlim([-x_max, x_max]);
ylim([-x_max, x_max]);
zlim([-x_max, x_max]);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

% 保存数据
save('C:\Users\zheyu\Desktop\Collision_Free_Formation_Quadrotor/Results/Data/estimated_xf_pos_valts.mat','est.xf_pos_valts');
save('Results/Data/estimated_xf_vel_valts.mat','est.xf_vel_valts');
save('Results/Data/desired_xf_pos_valts.mat','des.xf_pos_valts');
save('Results/Data/desired_xf_vel_valts.mat','des.xf_vel_valts');
save('Results/Data/actual_xf_pos_valts.mat','act.xf_pos_valts');
save('Results/Data/actual_xf_vel_valts.mat','act.xf_vel_valts');

save('Results/Data/actual_xl_pos_valts.mat','act.xl_pos_valts');
save('Results/Data/actual_xl_vel_valts.mat','act.xl_vel_valts');
