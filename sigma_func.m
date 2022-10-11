% syms x a1 a2 a3
% y = -a1*atan(a2*x+a3);
% y_dot = diff(y);
% z = 2*x - diff(y);
b = 5;
beta = 2;
max = (beta-1)*(b^2);

a1 = 50/pi;
a2 = 4;
a3 = 7;
syms x
y = -a1*atan(a2*x+a3);
y_dot = diff(y);
z = 2*x - diff(y);

y_fun = matlabFunction(y,'vars',x);
y_dot_fun = matlabFunction(y_dot,'vars',x);
z_fun = matlabFunction(z,'vars',x);

x1 = -1000:0.01:1000;
x2 = -sqrt(beta)*b:0.001:0;
% 第一个条件 递减 小于0
y_dot_value = y_dot_fun(x1);
% 第二个条件 有界
y_value = y_fun(x1);
% 第三个条件
z_value = z_fun(x2);

% plot
figure(1)
plot(x1,y_dot_value);


figure(2)
% plot(x1,max);
% hold on
plot(x1,y_value);
grid on
figure(3)
plot(x2,z_value);
grid on


% b = 5;
% beta = 2;
% max = (beta-1)*b^2;
% 
% k1 = 25;
% k2 = 5;
% k3 = 5;
% syms x
% y = k1*( (exp(-k2*x+k3)-1) / (exp(-k2*x+k3)+1) );
% % y = k1/(1+exp(k2*x+k3));
% y_dot = diff(y);
% z = 2*x - diff(y);
% 
% y_fun = matlabFunction(y,'vars',x);
% y_dot_fun = matlabFunction(y_dot,'vars',x);
% z_fun = matlabFunction(z,'vars',x);
% 
% x1 = -50:0.01:50;
% x2 = -sqrt(beta)*b:0.001:0;
% % 第一个条件 递减 小于0
% y_dot_value = y_dot_fun(x1);
% % 第二个条件 有界
% y_value = y_fun(x1);
% % 第三个条件
% z_value = z_fun(x2);
% % plot
% figure(1)
% plot(x1,y_dot_value);
% 
% 
% figure(2)
% % plot(x1,max);
% % hold on
% plot(x1,y_value);
% grid on
% figure(3)
% plot(x2,z_value);
% grid on