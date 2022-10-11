close all
clear
clc

addpath('C:\Users\zheyu\Desktop\Collision_Free_Formation_Quadrotor\utils\');
addpath('Control\');
addpath('Model\');
addpath('CBF_functions\');
addpath('Param\');
addpath('Plot_functions\');
% 生成参数
[~,obs,MA,param,obsCBF,collCBF] = Paramter();
% simulation time
fps = 200;
tF = 40;
tUnitStep = 1/fps;
tSteps = tF/tUnitStep;
taxis = linspace(0,tF,tSteps+1);
%% Obtstacle
obs.num = 5;
obs.pos = [0,10,1.4;
           0.2,14,1.5;
           -0.1,17,2.2;
           0,19,1.56;
           0,21,4];
obs.rad = 1.5;
%% PointMass parameters
param.range = 4;
%% Initial state
% position level
pos_Init = [0;0;0];
vel_Init = [0;0;0];
%% Varibale set for position level
% actual
pos_valts = [pos_Init,NaN(param.dim,tSteps)];
vel_valts = [vel_Init, NaN(param.dim,tSteps)];

% reference
ref_pos_valts = NaN(param.dim,tSteps);
ref_vel_valts = NaN(param.dim,tSteps);

%% Build control lyapunov function
[obj] = constraints_poscbf(param.A,param.B,obs.rad);
hwait=waitbar(0,'½øÐÐÖÐ>>>>>>>>>>');
%% Simulation Loop
for step = 1:tSteps
    t = taxis(step);
    % 获取reference的状态
    [ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap, Bd] = reference(t);
    Xd = [ref_pos,ref_vel,ref_acc,ref_jerk,ref_snap];
    ref_pos_valts(:,step) = ref_pos;
    % 获取当前时刻的状态
    x = pos_valts(:,step);
    v = vel_valts(:,step);
%     R = reshape(R_valts(:,step), 3, 3);
%     W = W_valts(:,step);
    X_now = [x;v];
    state = [x(1);v(1);x(2);v(2);x(3);v(3)];
    % 计算名义control
    control = pointMassCtrl(x,v,Xd,param);
    control_nom = control;
    %% CBF
    % 判断在探测范围内的障碍物
    [index_obs,dis_obs,flag_obs] = obstacleDetect(x,obs,param.range);
    if flag_obs == 1
        Con_A = [];
        Con_b = [];
        for k = 1:length(index_obs)
            obs_ind = obs.pos(index_obs(k),:)';
            lglf_cbf_ = obj.lglf_cbf(state,obs_ind);
            remaining_ = obj.remain_cbf(state,obs_ind);
            Con_A_temp = -lglf_cbf_;
            Con_b_temp = remaining_;
            %组合
            Con_A = [Con_A;Con_A_temp];
            Con_b = [Con_b;Con_b_temp];
        end
        options =  optimset('Display','notify');
        H = eye(param.dim);
        f_ = -eye(param.dim) * control_nom;
        [control, ~, exitflag, ~] = quadprog(H, f_, Con_A, Con_b, [], [], [], [], [], options);
    end
    % 传递f与M到四旋翼模型    
    [T, X_new] = ode45(@(t, x) pointModel(t, x, control), [0, tUnitStep], X_now, control);
    % update
    pos_valts(:,step+1) = X_new(end,1:3);
    vel_valts(:,step+1) = X_new(end,4:6);
    waitbar(step/tSteps,hwait,'simulating...');
end
%% Diagram
% error_1 = pos_valts(:,1:tSteps) - ref_pos_valts;
% error_2 = vel_valts(:,1:tSteps) - ref_vel_valts;
close(hwait);
figure(1)
% min_temp = min(min(pos_valts(1,:)), min(pos_valts(2,:)));
% lim_min = min(min_temp, min(pos_valts(3,:)));
% max_temp = max(max(pos_valts(1,:)), max(pos_valts(2,:)));
% lim_max = max(max_temp, max(pos_valts(2,:)));
for k = 1:obs.num
    obs_h = obstaclePlot(obs.pos(k,:),obs.rad); 
    hold on
end
plot3(ref_pos_valts(1,:),ref_pos_valts(2,:),ref_pos_valts(3,:),'r','LineWidth',1); hold on %参考轨迹用红色
plot3(pos_valts(1,:),pos_valts(2,:),pos_valts(3,:), 'k','LineWidth',1); %真实的轨迹用黑色
xlim([-3, 3]);
ylim([-10, 70]);
zlim([-8, 8]);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
 