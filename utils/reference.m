function [pos,vel,acc,jerk,snap,B] = reference(t)

% Attitude level
% b = [1;0;0];
% b_dot = [0;0;0];
% b_2dot = [0;0;0];
% B = [b,b_dot,b_2dot];
% Position level
pos = [0;0.75*t;1.5];
vel = [0;0.75;0];
acc = zeros(3,1);
jerk = zeros(3,1);
snap = zeros(3,1);
% % Attitude level
b = [cos(pi*t); sin(pi*t); 0];
b_dot = [-pi*sin(pi*t); pi*cos(pi*t); 0];
b_2dot = [-pi^2*cos(pi*t); -pi^2*sin(pi*t); 0];
B = [b,b_dot,b_2dot];
% % Position level
% pos = [0.4*t; 0.4*sin(pi*t); -0.6*cos(pi*t)];
% vel = [0.4; 0.4*pi*cos(pi*t); 0.6*pi*sin(pi*t)];
% acc = [0; -0.4*pi^2*sin(pi*t); 0.6*pi^2*cos(pi*t)];
% jerk = [0; -0.4*pi^3*cos(pi*t); -0.6*pi^3*sin(pi*t)];
% snap = [0; 0.4*pi^4*sin(pi*t); -0.6*pi^4*cos(pi*t)];



end


