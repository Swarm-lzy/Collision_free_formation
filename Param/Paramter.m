%% Parameter Function
function [T,obs,MA,param,obsCBF,collCBF] = Paramter()
% Simulation Time
T.fps = 200;
T.tF = 60;
T.tUnitStep = 1/T.fps;
T.tSteps = T.tF/T.tUnitStep;
T.taxis = linspace(0,T.tF,T.tSteps+1);

% obstacle parameter
obs.rad = 5;
obs.pos = [ 0,   15,  2;
           -2.9, 22,  0;
            3.0, 27,  1.5];
% obs.num = size(obs.pos,1);
obs.num = 3;
% Multiple quadrotor parameter
MA.N = 5;
MA.N_F = 4;
MA.N_L = 1;
MA.W = [0,0,0,0,0;
        1,0,0,0,0;
        0,1,0,0,0;
        0,0,1,0,0;
        0,0,0,1,1];
MA.formation_shape = [0,  3, -3,  0,   0; 
                      0,  0,  0,  3,  -3;
                      0,  0,  0,  0,   0];

% Quadrotor parameter
param.dim = 3;
param.m = 1;
param.e3 = [0;0;1];
param.g = 9.81;
param.J = diag([1,1,1]);
param.kp = 16*param.m;
param.kv = 5.6*param.m;
param.kr = 8.81;
param.kw = 2.54;
param.f_limit = 50;
param.M_limit = 2;
param.flag_limit = 0;
param.A = kron(eye(param.dim),[0,1;0,0]);
param.B = kron(eye(param.dim),[0;1]);
param.alpha_1 = 1;
param.alpha_2 = 5;
param.alpha_3 = 1;
param.beta_1 = 5;
param.beta_2 = 20;

% CBF parameter
obsCBF.beta = 1.2;
obsCBF.gamma = 1;
obsCBF.a1 = 3.1;
obsCBF.a2 = 4;
obsCBF.a3 = 5.5;
obsCBF.range = 8; %8
obsCBF.safeDis = obs.rad;

collCBF.gamma = 1;
collCBF.beta = 2;
collCBF.a1 = 2.5;
collCBF.a2 = 4;
collCBF.a3 = 5;
collCBF.range = 5; %3
collCBF.safeDis = 2;
end
