function [obs_h] = obstaclePlot(center,rad)
[x,y,z] = sphere;
x = rad*x;
y = rad*y;
z = rad*z;
a = center(1);
b = center(2);
c = center(3);
x = x+a;
y = y+b;
z = z+c;
obs_h = mesh(x,y,z);
end