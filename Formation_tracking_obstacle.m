clear workspace
close all
clear
clc
%% Time simulation
fps = 200;
tF = 60;
tUnitStep = 1/fps;
tSteps = tF/tUnitStep;
taxis = linspace(0,tF,tSteps+1);
%% Obtstacle
obs.num = 1;
obs.pos = [0,14,3;
           -2.9,29,1.5;
           3,36,3];
obs.rad = 5;
obsrange = 8;
%% quadrotor parameters
param.dim = 3;
param.m = 1;
param.e3 = [0;0;1];
param.g = 9.81;
param.J = diag([1,1,1]);
param.collrange = 3;
%% parameter need to be adjusted
beta = 1.2;
gamma = 1;
a1 = 3.1;
a2 = 4;
a3 = 5.5;
param.kp = 16*param.m;
param.kv = 5.6*param.m;
param.kr = 8.81;
param.kw = 2.54;
A = kron(eye(param.dim),[0,1;0,0]);
B = kron(eye(param.dim),[0;1]);
%% Multiple UAV
N_F = 6; %第2——7个
N_L = 1; %第1个
N = 7;
W = [0,0,0,0,0,0,0;
     1,0,0,0,0,0,1;
     0,1,0,0,0,0,0;
     0,0,1,0,0,0,0;
     0,0,0,1,0,0,0;
     0,0,0,0,1,0,1;
     0,0,0,0,0,1,0];
alpha_1 = 1;
alpha_2 = 5;
alpha_3 = 1;
beta_1 = 5;
beta_2 = 20;
v = 2;
Di = [0,  0,  0, -3,  3,  0,  0;
      0, -3,  3,  0,  0,  0,  0;
      0,  0,  0,  0,  0, -3, 3];

% myVideo = VideoWriter('myVideo.avi');
% myVideo.FrameRate = 25;
% open(myVideo);
%% Initial state
% 初始化真实的状态
act_xf_pos_Init = zeros(param.dim,N_F);
act_xf_vel_Init = zeros(param.dim,N_F);
act_xf_acc_Init = zeros(param.dim,N_F);
act_xf_jerk_Init = zeros(param.dim,N_F);
act_xf_snap_Init = zeros(param.dim,N_F);
act_xf_R_Init = [eye(3),eye(3),eye(3),eye(3),eye(3),eye(3)];
act_xf_W_Init = zeros(param.dim,N_F);
% 初始化估计的状态
rng(123)
est_xf_pos_Init = 2* rand(param.dim,N_F) * 3 - 3;
est_xf_vel_Init = 2* rand(param.dim,N_F) * 1 - 1;
est_xf_acc_Init = 2* rand(param.dim,N_F) * 1 - 1;
est_xf_jerk_Init =2* rand(param.dim,N_F) * 1 - 1;
est_xf_snap_Init =2* rand(param.dim,N_F) * 1 - 1;

%% 轨迹变量集合
% leader state trajectory
act_xl_pos_valts = NaN(N_L*param.dim,tSteps);
act_xl_vel_valts = NaN(N_L*param.dim,tSteps);
act_xl_acc_valts = NaN(N_L*param.dim,tSteps);
act_xl_jerk_valts = NaN(N_L*param.dim,tSteps);
act_xl_snap_valts = NaN(N_L*param.dim,tSteps);

% follower state trajectory
% desired follower's position and velocity
desired_xf_pos_valts = NaN(N_F*param.dim,tSteps);
desired_xf_vel_valts = NaN(N_F*param.dim,tSteps);
desired_xf_acc_valts = NaN(N_F*param.dim,tSteps);
desired_xf_jerk_valts = NaN(N_F*param.dim,tSteps);
desired_xf_snap_valts = NaN(N_F*param.dim,tSteps);

% estimated follower's position and velocity, 每个follower估计的desired state
estimated_xf_pos_valts = [reshape(est_xf_pos_Init,[],1), NaN(N_F*param.dim, tSteps)];
estimated_xf_vel_valts = [reshape(est_xf_vel_Init,[],1), NaN(N_F*param.dim, tSteps)];
estimated_xf_acc_valts = [reshape(est_xf_acc_Init,[],1), NaN(N_F*param.dim, tSteps)];
estimated_xf_jerk_valts = [reshape(est_xf_jerk_Init,[],1), NaN(N_F*param.dim, tSteps)];
estimated_xf_snap_valts = [reshape(est_xf_snap_Init,[],1), NaN(N_F*param.dim, tSteps)];

% actual follower's position and velocity
act_xf_pos_valts = [reshape(act_xf_pos_Init,[],1), NaN(N_F*param.dim, tSteps)];
act_xf_vel_valts = [reshape(act_xf_vel_Init,[],1), NaN(N_F*param.dim, tSteps)];
act_xf_acc_valts = [reshape(act_xf_acc_Init,[],1), NaN(N_F*param.dim, tSteps)];
act_xf_jerk_valts = [reshape(act_xf_jerk_Init,[],1), NaN(N_F*param.dim, tSteps)];
act_xf_snap_valts = [reshape(act_xf_snap_Init,[],1), NaN(N_F*param.dim, tSteps)];
act_xf_R_valts = [reshape(act_xf_R_Init,[],1), NaN(N_F*param.dim*param.dim, tSteps)];
act_xf_W_valts = [reshape(act_xf_W_Init,[],1), NaN(N_F*param.dim, tSteps);];

agent_info = zeros(N_F*param.dim,tSteps);
% [obj_cbf] = constraints_poscbf(A,B,Obs);
[obj] = constraints_cbf(param.dim,param.m,param.J,param.g,beta,obs.rad,param.e3,gamma,a1,a2,a3);
%% Build control barrier function
%% Simulation Loop
for step = 1:tSteps
    tnow = taxis(step);
    tnext = taxis(step+1);
    %% 获取领导者的状态
    [pos_l,vel_l,acc_l,jerk_l,snap_l, Bd] = leader_trajectory(tnow);
    Xd_l = [pos_l,vel_l,acc_l,jerk_l,snap_l];
    act_xl_pos_valts(:,step) = pos_l;
    act_xl_vel_valts(:,step) = vel_l;
    act_xl_acc_valts(:,step) = acc_l;
    act_xl_jerk_valts(:,step) = jerk_l;
    act_xl_snap_valts(:,step) = snap_l;

    for i = 1:N_F
        desired_xf_pos_valts(3*i-2:3*i,step) = pos_l + Di(:,i+1);
        desired_xf_vel_valts(3*i-2:3*i,step) = vel_l;
        desired_xf_acc_valts(3*i-2:3*i,step) = acc_l;
        desired_xf_jerk_valts(3*i-2:3*i,step) = jerk_l;
        desired_xf_snap_valts(3*i-2:3*i,step) = snap_l;
    end
    for i = 1:N_F
        error_vel = zeros(param.dim,1);
        error_pos = zeros(param.dim,1);
        error_acc = zeros(param.dim,1);
        error_jerk = zeros(param.dim,1);
        error_snap = zeros(param.dim,1);
        % snap
        for j = 1:N_F
            error_snap = error_snap + W(i+1,j+1)*(estimated_xf_snap_valts(3*i-2:3*i,step) - estimated_xf_snap_valts(3*j-2:3*j,step));
        end
        error_snap = error_snap + W(i+1,1)*(estimated_xf_snap_valts(3*i-2:3*i,step) - act_xl_snap_valts(:,step));
        dot_snap = - alpha_1 * error_snap - alpha_2 * sign(error_snap) - alpha_3 * diag((abs(error_snap))) * sign(error_snap);
        estimated_xf_snap_valts(3*i-2:3*i,step+1) = estimated_xf_snap_valts(3*i-2:3*i,step) + dot_snap * tUnitStep;
        % jerk
        for j = 1:N_F
            error_jerk = error_jerk + W(i+1,j+1)*(estimated_xf_jerk_valts(3*i-2:3*i,step) - estimated_xf_jerk_valts(3*j-2:3*j,step));
        end
        error_jerk = error_jerk + W(i+1,1)*(estimated_xf_jerk_valts(3*i-2:3*i,step) - act_xl_jerk_valts(:,step));
        dot_jerk = - alpha_1 * error_jerk - alpha_2 * sign(error_jerk) - alpha_3 * diag((abs(error_jerk))) * sign(error_jerk);
        estimated_xf_jerk_valts(3*i-2:3*i,step+1) = estimated_xf_jerk_valts(3*i-2:3*i,step) + dot_jerk * tUnitStep;
        % acceleration
        for j = 1:N_F
            error_acc = error_acc + W(i+1,j+1)*(estimated_xf_acc_valts(3*i-2:3*i,step) - estimated_xf_acc_valts(3*j-2:3*j,step));
        end
        error_acc = error_acc + W(i+1,1)*(estimated_xf_acc_valts(3*i-2:3*i,step) - act_xl_acc_valts(:,step));
        dot_acc = - alpha_1 * error_acc - alpha_2 * sign(error_acc) - alpha_3 * diag((abs(error_acc))) * sign(error_acc);
        estimated_xf_acc_valts(3*i-2:3*i,step+1) = estimated_xf_acc_valts(3*i-2:3*i,step) + dot_acc * tUnitStep;
        % velocity
        for j = 1:N_F
            error_vel = error_vel + W(i+1,j+1)*(estimated_xf_vel_valts(3*i-2:3*i,step) - estimated_xf_vel_valts(3*j-2:3*j,step));
        end
        error_vel = error_vel + W(i+1,1)*(estimated_xf_vel_valts(3*i-2:3*i,step) - act_xl_vel_valts(:,step));
        dot_vel = - alpha_1 * error_vel - alpha_2 * sign(error_vel) - alpha_3 * diag((abs(error_vel))) * sign(error_vel);
        estimated_xf_vel_valts(3*i-2:3*i,step+1) = estimated_xf_vel_valts(3*i-2:3*i,step) + dot_vel * tUnitStep;
        % position
        for j = 1:N_F
            error_pos = error_pos + W(i+1,j+1)*(estimated_xf_pos_valts(3*i-2:3*i,step) - estimated_xf_pos_valts(3*j-2:3*j,step) - (Di(:,i+1) - Di(:,j+1)));
        end
        error_pos = error_pos + W(i+1,1)*(estimated_xf_pos_valts(3*i-2:3*i,step) - act_xl_pos_valts(:,step) - (Di(:,i+1) - Di(:,1)));
        dot_pos = estimated_xf_vel_valts(3*i-2:3*i,step) - beta_1 * error_pos - beta_2 * sign(error_pos);
        estimated_xf_pos_valts(3*i-2:3*i,step+1) = estimated_xf_pos_valts(3*i-2:3*i,step) + dot_pos * tUnitStep;
    end
    %% 获取所有的无人机的真实位置
        %% 跟随者的轨迹跟踪
    for i = 1:N_F
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
        % 障碍物
        [index_obs,dis_obs,flag_obs,index_agent,dis_agent,flag_coll] = judgeCollision(i,act_pos,obs,act_xf_pos_valts(:,step),N_F,obsrange,param.collrange);
        if flag_obs == 1
            Con_A = [];
            Con_b = [];
            for k = 1:length(index_obs)
                obs_ind = obs.pos(index_obs(k),:)';
                obs_vel = zeros(param.dim,1);
                TAU_1 = obj.TAU_1_func(act_pos,act_vel,act_R,act_W,obs_ind,obs_vel);
                TAU_2 = obj.TAU_2_func(act_pos,act_vel,act_R,act_W,obs_ind,obs_vel);
                remaining = obj.remaining_func(act_pos,act_vel,act_R,act_W,obs_ind,obs_vel);
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
        [~, X_new] = ode45(@(t, x) UAVModel(param, t, x, control), [0, tUnitStep], act_state, control);
        act_xf_pos_valts(3*i-2:3*i,step+1) = X_new(end,1:3);
        act_xf_vel_valts(3*i-2:3*i,step+1) = X_new(end,4:6);
        act_xf_R_valts(9*i-8:9*i,step+1) = X_new(end,7:15);
        act_xf_W_valts(3*i-2:3*i,step+1) = X_new(end,16:18);
    end

end
% Compare the desired trajectory and estiamted trajectory
figure(1)
plot3(desired_xf_pos_valts(1,:),desired_xf_pos_valts(2,:),desired_xf_pos_valts(3,:),'r','LineWidth',1);
hold on
plot3(estimated_xf_pos_valts(1,:),estimated_xf_pos_valts(2,:),estimated_xf_pos_valts(3,:),'b','LineWidth',1);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

figure(2)
plot3(desired_xf_pos_valts(4,:),desired_xf_pos_valts(5,:),desired_xf_pos_valts(6,:),'r','LineWidth',1);
hold on
plot3(estimated_xf_pos_valts(4,:),estimated_xf_pos_valts(5,:),estimated_xf_pos_valts(6,:),'b','LineWidth',1);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

figure(3)
plot3(desired_xf_pos_valts(7,:),desired_xf_pos_valts(8,:),desired_xf_pos_valts(9,:),'r','LineWidth',1);
hold on
plot3(estimated_xf_pos_valts(7,:),estimated_xf_pos_valts(8,:),estimated_xf_pos_valts(9,:),'b','LineWidth',1);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

figure(4)
plot3(desired_xf_pos_valts(10,:),desired_xf_pos_valts(11,:),desired_xf_pos_valts(12,:),'r','LineWidth',1);
hold on
plot3(estimated_xf_pos_valts(10,:),estimated_xf_pos_valts(11,:),estimated_xf_pos_valts(12,:),'b','LineWidth',1);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
% 
% figure(5)
% plot3(desired_xf_pos_valts(13,:),desired_xf_pos_valts(14,:),desired_xf_pos_valts(15,:),'r','LineWidth',1);
% hold on
% plot3(estimated_xf_pos_valts(13,:),estimated_xf_pos_valts(14,:),estimated_xf_pos_valts(15,:),'b','LineWidth',1);
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')
% 
% figure(6)
% plot3(desired_xf_pos_valts(16,:),desired_xf_pos_valts(17,:),desired_xf_pos_valts(18,:),'r','LineWidth',1);
% hold on
% plot3(estimated_xf_pos_valts(16,:),estimated_xf_pos_valts(17,:),estimated_xf_pos_valts(18,:),'b','LineWidth',1);
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')


figure(7)
plot3(act_xf_pos_valts(1,:), act_xf_pos_valts(2,:), act_xf_pos_valts(3,:),'b','LineWidth',1);
hold on
plot3(act_xf_pos_valts(4,:),act_xf_pos_valts(5,:),act_xf_pos_valts(6,:),'r','LineWidth',1);
hold on
plot3(act_xf_pos_valts(7,:),act_xf_pos_valts(8,:),act_xf_pos_valts(9,:),'k','LineWidth',1);
hold on
plot3(act_xf_pos_valts(10,:),act_xf_pos_valts(11,:),act_xf_pos_valts(12,:),'m','LineWidth',1);
hold on
plot3(act_xf_pos_valts(13,:),act_xf_pos_valts(14,:),act_xf_pos_valts(15,:),'c','LineWidth',1);
hold on
plot3(act_xf_pos_valts(16,:),act_xf_pos_valts(17,:),act_xf_pos_valts(18,:),'g','LineWidth',1);
hold on
for k = 1:obs.num
    obs_h = obstaclePlot(obs.pos(k,:),obs.rad); 
    hold on
end
x_max = max(ceil(abs(act_xl_pos_valts(:))), [], 'all');
xlim([-x_max, x_max]);
ylim([-x_max, x_max]);
zlim([-x_max, x_max]);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

pos_error = desired_xf_pos_valts - estimated_xf_pos_valts(:,1:tSteps);
vel_error = desired_xf_vel_valts - estimated_xf_vel_valts(:,1:tSteps);
acc_error = desired_xf_acc_valts - estimated_xf_acc_valts(:,1:tSteps);
jerk_error = desired_xf_jerk_valts - estimated_xf_jerk_valts(:,1:tSteps);
snap_error = desired_xf_snap_valts - estimated_xf_snap_valts(:,1:tSteps);

% 动态运动轨迹
% trajectory = vertcat(act_xf_pos_valts(:,1:tSteps),act_xl_pos_valts);
% X = reshape(trajectory, param.dim, N, tSteps);
% fig = figure('Name', 'Formation Trajectory', 'Color', 'white', 'Position', [50 50 1280 720]);%, 'visible', 'off');
% set(gca,'TickLabelInterpreter', 'latex');
% grid on
% hold on
% scthf = scatter3(X(1,1:6,1), X(2,1:6,1), X(3,1:6,1), 10, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [164 0 6]/255, 'MarkerFaceAlpha', .9);%'MarkerFaceColor', 'none');
% scthl = scatter3(X(1,7,1), X(2,7,1), X(3,7,1), 40, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [6 0 164]/255, 'MarkerFaceAlpha', .9);%'MarkerFaceColor', 'none');
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
%     set(scthf, 'XData', X(1,1:6,fii), 'YData', X(2,1:6,fii), 'ZData',X(3,1:6,fii));
%     set(scthl, 'XData', X(1,7,fii), 'YData', X(2,7,fii), 'ZData',X(3,7,fii));
%     temp = getframe(gcf);
%     frame = temp.cdata;
%     writeVideo(myVideo,frame);
%     pause(tUnitStep/2)
% end
% close(myVideo);

