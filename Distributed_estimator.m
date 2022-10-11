%% Distributed estimator for multi agent system
close all
clear
clc

Param.N_F = 4;
Param.N_L = 1;
Param.N = 5;
Param.dim = 3;

W = [0,0,0,0,0;
     1,0,0,0,0;
     0,1,0,0,0;
     0,0,1,0,0;
     0,0,0,1,1];
% Simulation Time
fps = 400;
tF = 10;
tUnitStep = 1/fps;
tSteps = tF/tUnitStep;
taxis = linspace(0,tF,tSteps+1);

% Estimator Parameters
alpha_1 = 1;
alpha_2 = 5;
alpha_3 = 1;
beta_1 = 5;
beta_2 = 20;
Di = [0,  0,   0,  -30,  30; 
      0, -30,  30,  0,   0;
      0,  0,   0,   0,   0];

%% 初始化状态
estimated_xf_pos_Init = 2*rand(Param.dim,Param.N_F) * 10 -10;
estimated_xf_vel_Init = 2*rand(Param.dim,Param.N_F) * 1 -1;
%% 状态轨迹变量
act_xl_pos_valts = NaN(Param.N_L*Param.dim,tSteps);
act_xl_vel_valts = NaN(Param.N_L*Param.dim,tSteps);

desired_xf_pos_valts = NaN(Param.N_F*Param.dim,tSteps);
desired_xf_vel_valts = NaN(Param.N_F*Param.dim,tSteps);
estimated_xf_pos_valts = [reshape(estimated_xf_pos_Init,[],1), NaN(Param.N_F * Param.dim, tSteps)];
estimated_xf_vel_valts = [reshape(estimated_xf_vel_Init,[],1), NaN(Param.N_F * Param.dim, tSteps)];

hwait=waitbar(0,'½øÐÐÖÐ>>>>>>>>>>');
for step = 1:tSteps
    tnow = taxis(step);
    tnext = taxis(step+1);
    [ref_pos,ref_vel,~,~,~,~] = leader_trajectory(tnow);
    act_xl_pos_valts(:,step) = ref_pos;
    act_xl_vel_valts(:,step) = ref_vel;
    for i = 1:Param.N_F
        desired_xf_pos_valts(3*i-2:3*i,step) = ref_pos + Di(:,i+1);
        desired_xf_vel_valts(3*i-2:3*i,step) = ref_vel;
    end
    for i = 1:Param.N_F
        error_vel = zeros(Param.dim,1);
        error_pos = zeros(Param.dim,1);
        %% Trajectory Generation
        % velocity
        for j = 1:Param.N_F
            error_vel = error_vel + W(i+1,j+1)*(estimated_xf_vel_valts(3*i-2:3*i,step) - estimated_xf_vel_valts(3*j-2:3*j,step));
        end
        error_vel = error_vel + W(i+1,1)*(estimated_xf_vel_valts(3*i-2:3*i,step) - act_xl_vel_valts(:,step));
%         dot_vel = - alpha_2 * sign(error_vel);
        dot_vel = - alpha_1 * error_vel - alpha_2 * sign(error_vel); 
%         dot_vel = - alpha_1 * error_vel - alpha_3 * diag((abs(error_vel))) * sign(error_vel);
%         dot_vel = - alpha_1 * error_vel - alpha_2 * sign(error_vel) - alpha_3 * diag((abs(error_vel))) * sign(error_vel);
%         dot_vel = - alpha_2 * sign(error_vel) - alpha_3 * diag((abs(error_vel))) * sign(error_vel) ;
%         dot_vel = - error_vel - alpha_2 * (tanh(error_vel));
        estimated_xf_vel_valts(3*i-2:3*i,step+1) = estimated_xf_vel_valts(3*i-2:3*i,step) + dot_vel * tUnitStep;

        % position
        for j = 1:Param.N_F
            error_pos = error_pos + W(i+1,j+1)*(estimated_xf_pos_valts(3*i-2:3*i,step) - estimated_xf_pos_valts(3*j-2:3*j,step) - (Di(:,i+1) - Di(:,j+1)));
        end
        error_pos = error_pos + W(i+1,1)*(estimated_xf_pos_valts(3*i-2:3*i,step) - act_xl_pos_valts(:,step) - (Di(:,i+1) - Di(:,1)));
        dot_pos = estimated_xf_vel_valts(3*i-2:3*i,step) - beta_1 * error_pos - beta_2 * sign(error_pos);
%         dot_pos = estimated_xf_vel_valts(3*i-2:3*i,step) - error_pos;
%         dot_pos = estimated_xf_vel_valts(3*i-2:3*i,step) - error_pos - sigma * sign(error_pos) - sigma * diag((abs(error_pos))) * sign(error_pos);
%         dot_pos = estimated_xf_vel_valts(3*i-2:3*i,step) - error_pos - sigma * diag((abs(error_vel))) * sign(error_vel);
%         dot_pos = estimated_xf_vel_valts(3*i-2:3*i,step) - sigma * sign(error_pos);
%         dot_pos = estimated_xf_vel_valts(3*i-2:3*i,step) - sigma * tanh(error_pos);
        estimated_xf_pos_valts(3*i-2:3*i,step+1) = estimated_xf_pos_valts(3*i-2:3*i,step) + dot_pos * tUnitStep;
    end
    waitbar(step/tSteps,hwait,'simulating...');
end
close(hwait);
% 计算估计误差
pos_error = estimated_xf_pos_valts(:,1:tSteps) - desired_xf_pos_valts;
vel_error = estimated_xf_vel_valts(:,1:tSteps) - desired_xf_vel_valts;
figure % 绘制位置估计误差
subplot(3,1,1) % x坐标
for i = 1:Param.N_F
    plot(taxis(1:tSteps),pos_error(3*i-2,:));
    hold on
end
subplot(3,1,2) % y坐标
for i = 1:Param.N_F
    plot(taxis(1:tSteps),pos_error(3*i-1,:));
    hold on
end
subplot(3,1,3) % z坐标
for i = 1:Param.N_F
    plot(taxis(1:tSteps),pos_error(3*i,:));
    hold on
end
figure % 绘制速度估计误差
subplot(3,1,1) % x坐标
for i = 1:Param.N_F
    plot(taxis(1:tSteps),vel_error(3*i-2,:));
    hold on
end
subplot(3,1,2) % y坐标
for i = 1:Param.N_F
    plot(taxis(1:tSteps),vel_error(3*i-1,:));
    hold on
end
subplot(3,1,3) % z坐标
for i = 1:Param.N_F
    plot(taxis(1:tSteps),vel_error(3*i,:));
    hold on
end
CurrentFolder = pwd;
save('C:\Users\zheyu\Desktop\Collision_Free_Formation_Quadrotor/Data/estimated_xf_pos.mat','estimated_xf_pos_valts');
save('C:\Users\zheyu\Desktop\Collision_Free_Formation_Quadrotor/Data/estimated_xf_vel.mat','estimated_xf_vel_valts');