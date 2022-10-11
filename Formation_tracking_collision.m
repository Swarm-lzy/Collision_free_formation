close all
clear
clc
% 添加必要的函数
addpath('utils\');
addpath('Param\');
addpath('CBF_functions\');
addpath('Control\');
addpath('Model\');
addpath('Plot_functions\');
addpath('quadrotor_model\');
% parameter
[T,obs,MA,param,obsCBF,collCBF] = Paramter();
% Initial state
% 实际状态
% act_xf_pos_Init = zeros(param.dim,N_F);
act_xf_pos_Init = [0,  0,  3,  -3;
                  -3,  3,  0,  0;
                   0,  0,  0,  0];
act_xf_vel_Init = zeros(param.dim,MA.N_F);
act_xf_acc_Init = zeros(param.dim,MA.N_F);
act_xf_jerk_Init = zeros(param.dim,MA.N_F);
act_xf_snap_Init = zeros(param.dim,MA.N_F);
act_xf_R_Init = [eye(3),eye(3),eye(3),eye(3)];
act_xf_W_Init = zeros(param.dim,MA.N_F);
% 估计状态
rng(123)
est_xf_pos_Init = 2* rand(param.dim,MA.N_F) * 3 - 3;
est_xf_vel_Init = rand(param.dim,MA.N_F);
est_xf_acc_Init = rand(param.dim,MA.N_F);
est_xf_jerk_Init = rand(param.dim,MA.N_F);
est_xf_snap_Init = rand(param.dim,MA.N_F);

%% 轨迹变量集合
act_xl_pos_valts = NaN(MA.N_L*param.dim,T.tSteps);
act_xl_vel_valts = NaN(MA.N_L*param.dim,T.tSteps);
act_xl_acc_valts = NaN(MA.N_L*param.dim,T.tSteps);
act_xl_jerk_valts = NaN(MA.N_L*param.dim,T.tSteps);
act_xl_snap_valts = NaN(MA.N_L*param.dim,T.tSteps);

desired_xf_pos_valts = NaN(MA.N_F*param.dim,T.tSteps);
desired_xf_vel_valts = NaN(MA.N_F*param.dim,T.tSteps);
desired_xf_acc_valts = NaN(MA.N_F*param.dim,T.tSteps);
desired_xf_jerk_valts = NaN(MA.N_F*param.dim,T.tSteps);
desired_xf_snap_valts = NaN(MA.N_F*param.dim,T.tSteps);

estimated_xf_pos_valts = [reshape(est_xf_pos_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
estimated_xf_vel_valts = [reshape(est_xf_vel_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
estimated_xf_acc_valts = [reshape(est_xf_acc_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
estimated_xf_jerk_valts = [reshape(est_xf_jerk_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
estimated_xf_snap_valts = [reshape(est_xf_snap_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];

act_xf_pos_valts = [reshape(act_xf_pos_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act_xf_vel_valts = [reshape(act_xf_vel_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act_xf_acc_valts = [reshape(act_xf_acc_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act_xf_jerk_valts = [reshape(act_xf_jerk_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act_xf_snap_valts = [reshape(act_xf_snap_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps)];
act_xf_R_valts = [reshape(act_xf_R_Init,[],1), NaN(MA.N_F*param.dim*param.dim, T.tSteps)];
act_xf_W_valts = [reshape(act_xf_W_Init,[],1), NaN(MA.N_F*param.dim, T.tSteps);];

[obj_coll] = constraints_cbf(param, collCBF);
Flag_coll = zeros(MA.N_F,MA.N_F,T.tSteps);
Thdvalue = [];
% [obj_coll] = constraints_cbf(param.m,param.J,param.g,coll_beta,param.safeDis,param.e3,coll_gamma,coll_a1,coll_a2,coll_a3);
hwait=waitbar(0,'½øÐÐÖÐ>>>>>>>>>>');
for step = 1:T.tSteps
    tnow = T.taxis(step);
    tnext = T.taxis(step+1);
    %% 获取领导者的状态
    [pos_l,vel_l,acc_l,jerk_l,snap_l, Bd] = leader_trajectory(tnow);
    act_xl_pos_valts(:,step) = pos_l;
    act_xl_vel_valts(:,step) = vel_l;
    act_xl_acc_valts(:,step) = acc_l;
    act_xl_jerk_valts(:,step) = jerk_l;
    act_xl_snap_valts(:,step) = snap_l;

    for i = 1:MA.N_F
        desired_xf_pos_valts(3*i-2:3*i,step) = pos_l + MA.formation_shape(:,i+1);
        desired_xf_vel_valts(3*i-2:3*i,step) = vel_l;
        desired_xf_acc_valts(3*i-2:3*i,step) = acc_l;
        desired_xf_jerk_valts(3*i-2:3*i,step) = jerk_l;
        desired_xf_snap_valts(3*i-2:3*i,step) = snap_l;
    end
    for i = 1:MA.N_F
        error_vel = zeros(param.dim,1);
        error_pos = zeros(param.dim,1);
        error_acc = zeros(param.dim,1);
        error_jerk = zeros(param.dim,1);
        error_snap = zeros(param.dim,1);
        % snap
        for j = 1:MA.N_F
            error_snap = error_snap + MA.W(i+1,j+1)*(estimated_xf_snap_valts(3*i-2:3*i,step) - estimated_xf_snap_valts(3*j-2:3*j,step));
        end
        error_snap = error_snap + MA.W(i+1,1)*(estimated_xf_snap_valts(3*i-2:3*i,step) - act_xl_snap_valts(:,step));
        dot_snap = - param.alpha_1 * error_snap - param.alpha_2 * sign(error_snap) - param.alpha_3 * diag((abs(error_snap))) * sign(error_snap);
        estimated_xf_snap_valts(3*i-2:3*i,step+1) = estimated_xf_snap_valts(3*i-2:3*i,step) + dot_snap * T.tUnitStep;
        % jerk
        for j = 1:MA.N_F
            error_jerk = error_jerk + MA.W(i+1,j+1)*(estimated_xf_jerk_valts(3*i-2:3*i,step) - estimated_xf_jerk_valts(3*j-2:3*j,step));
        end
        error_jerk = error_jerk + MA.W(i+1,1)*(estimated_xf_jerk_valts(3*i-2:3*i,step) - act_xl_jerk_valts(:,step));
        dot_jerk = - param.alpha_1 * error_jerk - param.alpha_2 * sign(error_jerk) - param.alpha_3 * diag((abs(error_jerk))) * sign(error_jerk);
        estimated_xf_jerk_valts(3*i-2:3*i,step+1) = estimated_xf_jerk_valts(3*i-2:3*i,step) + dot_jerk * T.tUnitStep;
        % acceleration
        for j = 1:MA.N_F
            error_acc = error_acc + MA.W(i+1,j+1)*(estimated_xf_acc_valts(3*i-2:3*i,step) - estimated_xf_acc_valts(3*j-2:3*j,step));
        end
        error_acc = error_acc + MA.W(i+1,1)*(estimated_xf_acc_valts(3*i-2:3*i,step) - act_xl_acc_valts(:,step));
        dot_acc = - param.alpha_1 * error_acc - param.alpha_2 * sign(error_acc) - param.alpha_3 * diag((abs(error_acc))) * sign(error_acc);
        estimated_xf_acc_valts(3*i-2:3*i,step+1) = estimated_xf_acc_valts(3*i-2:3*i,step) + dot_acc * T.tUnitStep;
        % velocity
        for j = 1:MA.N_F
            error_vel = error_vel + MA.W(i+1,j+1)*(estimated_xf_vel_valts(3*i-2:3*i,step) - estimated_xf_vel_valts(3*j-2:3*j,step));
        end
        error_vel = error_vel + MA.W(i+1,1)*(estimated_xf_vel_valts(3*i-2:3*i,step) - act_xl_vel_valts(:,step));
        dot_vel = - param.alpha_1 * error_vel - param.alpha_2 * sign(error_vel) - param.alpha_3 * diag((abs(error_vel))) * sign(error_vel);
        estimated_xf_vel_valts(3*i-2:3*i,step+1) = estimated_xf_vel_valts(3*i-2:3*i,step) + dot_vel * T.tUnitStep;
        % position
        for j = 1:MA.N_F
            error_pos = error_pos + MA.W(i+1,j+1)*(estimated_xf_pos_valts(3*i-2:3*i,step) - estimated_xf_pos_valts(3*j-2:3*j,step) - (MA.formation_shape(:,i+1) - MA.formation_shape(:,j+1)));
        end
        error_pos = error_pos + MA.W(i+1,1)*(estimated_xf_pos_valts(3*i-2:3*i,step) - act_xl_pos_valts(:,step) - (MA.formation_shape(:,i+1) - MA.formation_shape(:,1)));
        dot_pos = estimated_xf_vel_valts(3*i-2:3*i,step) - param.beta_1 * error_pos - param.beta_2 * sign(error_pos);
        estimated_xf_pos_valts(3*i-2:3*i,step+1) = estimated_xf_pos_valts(3*i-2:3*i,step) + dot_pos * T.tUnitStep;
    end
        %% 跟随者的轨迹跟踪
    for i = 1:MA.N_F
        ref_pos = estimated_xf_pos_valts(3*i-2:3*i,step);
        ref_vel = estimated_xf_vel_valts(3*i-2:3*i,step);
        ref_acc = estimated_xf_acc_valts(3*i-2:3*i,step);
        ref_jerk = estimated_xf_jerk_valts(3*i-2:3*i,step);
        ref_snap = estimated_xf_snap_valts(3*i-2:3*i,step);
        Xd_f = [ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap];
        act_pos = act_xf_pos_valts(3*i-2:3*i,step);
        act_vel = act_xf_vel_valts(3*i-2:3*i,step);
        act_R_temp = act_xf_R_valts(9*i-8:9*i,step);
        act_R = reshape(act_R_temp,3,3);
        act_W = act_xf_W_valts(3*i-2:3*i,step);
        act_state = [act_pos;act_vel;act_R_temp;act_W];
        f = posCtrl(param.kp, param.kv, param.m, param.g, param.e3, act_R, act_pos, act_vel, Xd_f);
        [~,~,~,M] = mntCtrl(param, act_state, Xd_f, Bd);
        control = [f;M];
        control_nom = control;
        % 判断碰撞
        [index_obs,dis_obs,flag_obs] = obstacleDetect(act_pos,obs,obsCBF.range);
        [index_coll,dis_coll,flag_coll] = collisionDetect(i,act_pos,act_xf_pos_valts(:,step),collCBF.range);
%         [~,~,~,index_agent,dis_agent,flag_coll] = judgeCollision(i, act_pos, obs, act_xf_pos_valts(:,step), MA.N_F, obsCBF.range, collCBF.range);
        Flag_coll(i,:,step) = flag_coll;
        if ~isempty(index_obs)
            Con_A = [];
            Con_b = [];
            for k = 1:length(index_coll)
                agent_index = index_coll(k);
                agent_pos = act_xf_pos_valts(3*agent_index-2:3*agent_index,step);
                agent_vel = act_xf_vel_valts(3*agent_index-2:3*agent_index,step);
                TAU_1_coll = obj_coll.TAU_1_func(act_pos,act_vel,act_R,act_W,agent_pos,agent_vel);
                TAU_2_coll = obj_coll.TAU_2_func(act_pos,act_vel,act_R,act_W,agent_pos,agent_vel);
                remaining_coll = obj_coll.remaining_func(act_pos,act_vel,act_R,act_W,agent_pos,agent_vel);
                thdvalue = obj_coll.thd_func(act_pos,act_vel,act_R,act_W,agent_pos,agent_vel);
                Thdvalue = [Thdvalue,thdvalue];
                Con_A_coll = -[TAU_1_coll,TAU_2_coll];
                Con_b_coll = remaining_coll;
            end
            Con_A = [Con_A;Con_A_coll];
            Con_b = [Con_b;Con_b_coll];
            options =  optimset('Display','notify');
            H = eye(4);
            f_ = -eye(4) * control_nom;
            [control, ~, exitflag, ~] = quadprog(H, f_, Con_A, Con_b, [], [], [], [], [], options);
        end
        [~, X_new] = ode45(@(t, x) UAVModel(param, t, x, control), [0, T.tUnitStep], act_state, control);
        act_xf_pos_valts(3*i-2:3*i,step+1) = X_new(end,1:3);
        act_xf_vel_valts(3*i-2:3*i,step+1) = X_new(end,4:6);
        act_xf_R_valts(9*i-8:9*i,step+1) = X_new(end,7:15);
        act_xf_W_valts(3*i-2:3*i,step+1) = X_new(end,16:18);
    end
    waitbar(step/T.tSteps,hwait,'simulating...');
end
close(hwait);
figure(1)
% first quadrotor blue
plot3(act_xf_pos_valts(1,:), act_xf_pos_valts(2,:), act_xf_pos_valts(3,:),'b','LineWidth',1);
hold on
% second quadrotor red
plot3(act_xf_pos_valts(4,:),act_xf_pos_valts(5,:),act_xf_pos_valts(6,:),'r','LineWidth',1);
hold on
% third quadrotor black
plot3(act_xf_pos_valts(7,:),act_xf_pos_valts(8,:),act_xf_pos_valts(9,:),'k','LineWidth',1);
hold on
% fourth quadrotor magenta
plot3(act_xf_pos_valts(10,:),act_xf_pos_valts(11,:),act_xf_pos_valts(12,:),'m','LineWidth',1);
hold on
% for k = 1:obs.num
%     obs_h = obstaclePlot(obs.pos(k,:),obs.rad); 
%     hold on
% end
x_max = max(ceil(abs(act_xl_pos_valts(:))), [], 'all');
xlim([-x_max, x_max]);
ylim([-x_max, x_max]);
zlim([-x_max, x_max]);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

% follower trajectory error
error_12 = act_xf_pos_valts(1:3,:) - act_xf_pos_valts(4:6,:);
error_13 = act_xf_pos_valts(1:3,:) - act_xf_pos_valts(7:9,:);
error_14 = act_xf_pos_valts(1:3,:) - act_xf_pos_valts(10:12,:);
% error_15 = act_xf_pos_valts(1:3,:) - act_xf_pos_valts(13:15,:);
% error_16 = act_xf_pos_valts(1:3,:) - act_xf_pos_valts(16:18,:);
error_23 = act_xf_pos_valts(4:6,:) - act_xf_pos_valts(7:9,:);
error_24 = act_xf_pos_valts(4:6,:) - act_xf_pos_valts(10:12,:);
% error_25 = act_xf_pos_valts(4:6,:) - act_xf_pos_valts(13:15,:);
% error_26 = act_xf_pos_valts(4:6,:) - act_xf_pos_valts(16:18,:);
error_34 = act_xf_pos_valts(7:9,:) - act_xf_pos_valts(10:12,:);
% error_35 = act_xf_pos_valts(7:9,:) - act_xf_pos_valts(13:15,:);
% error_36 = act_xf_pos_valts(7:9,:) - act_xf_pos_valts(16:18,:);
% error_45 = act_xf_pos_valts(10:12,:) - act_xf_pos_valts(13:15,:);
% error_46 = act_xf_pos_valts(10:12,:) - act_xf_pos_valts(16:18,:);
% error_56 = act_xf_pos_valts(13:15,:) - act_xf_pos_valts(16:18,:);
error_12_norm = zeros(T.tSteps,1);
error_13_norm = zeros(T.tSteps,1);
error_14_norm = zeros(T.tSteps,1);
error_23_norm = zeros(T.tSteps,1);
error_24_norm = zeros(T.tSteps,1);
error_34_norm = zeros(T.tSteps,1);
for i = 1:T.tSteps
    error_12_norm(i) = norm(error_12(:,i));
    error_13_norm(i) = norm(error_13(:,i));
    error_14_norm(i) = norm(error_14(:,i));
    error_23_norm(i) = norm(error_23(:,i));
    error_24_norm(i) = norm(error_24(:,i));
    error_34_norm(i) = norm(error_34(:,i));
end

% trajectory = vertcat(act_xf_pos_valts(:,1:tSteps),act_xl_pos_valts);
% X = reshape(trajectory, param.dim, N, tSteps);
% fig = figure('Name', 'Formation Trajectory', 'Color', 'white', 'Position', [50 50 1280 720]);%, 'visible', 'off');
% set(gca,'TickLabelInterpreter', 'latex');
% grid on
% hold on
% scthf = scatter3(X(1,1:4,1), X(2,1:4,1), X(3,1:4,1), 10, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [164 0 6]/255, 'MarkerFaceAlpha', .9);%'MarkerFaceColor', 'none');
% scthl = scatter3(X(1,5,1), X(2,5,1), X(3,5,1), 40, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [6 0 164]/255, 'MarkerFaceAlpha', .9);%'MarkerFaceColor', 'none');
% % obsh = scatter3(obs.pos(:,1), obs.pos(:,2),obs.pos(:,3), 200, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [25 12 120]/255, 'MarkerFaceAlpha', .9); 
% for k = 1:obs.num
%     obs_h = obstaclePlot(obs.pos(k,:),obs.rad); 
% %     scatter3(obs.pos(k,1),obs.pos(k,2),obs.pos(k,3));
%     hold on
% end
% % obth = scatter3(obs.pos(:,1), obs.pos(:,2), obs.pos(:,3), 500, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [25 12 120]/255, 'MarkerFaceAlpha', .9); 
% 
% axis('tight', 'manual')
% max_l = max( ceil(abs( trajectory(:) )) ,[], 'all');
% axis([-1 1 -1 1 -1 1] * max_l);
% view(45,60)
% 
%   
% xlabel('$$x (t)$$', 'Interpreter', 'latex');
% ylabel('$$y (t)$$', 'Interpreter', 'latex');
% zlabel('$$z (t)$$', 'Interpreter', 'latex');
% pause(tUnitStep/2)
% 
% for fii = 1:tSteps
%     set(scthf, 'XData', X(1,1:4,fii), 'YData', X(2,1:4,fii), 'ZData',X(3,1:4,fii));
%     set(scthl, 'XData', X(1,5,fii), 'YData', X(2,5,fii), 'ZData',X(3,5,fii));
% %     temp = getframe(gcf);
% %     frame = temp.cdata;
% %     writeVideo(myVideo,frame);
%     pause(tUnitStep/2)
% end
% % close(myVideo);





























