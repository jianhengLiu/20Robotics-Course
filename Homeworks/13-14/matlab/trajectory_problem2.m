function [p,v,a] = trajectory_problem2()
theta0 = 5;
thetav = 15;
thetag = -10;

td = 2;

subplot(3,1,1)
plot(0,theta0,'r*');
hold on;
plot(td,thetav,'r*');
hold on;
plot(td+td,thetag,'r*');
hold on;

v0 = 0;
vv = 0;
vg = 0;

h0v = thetav - theta0;
a01 = theta0;
a11 = v0;
a21 = (3*h0v-(2*v0+vv)*td)/td^2;
a31 = (-2*h0v+(v0+vv)*td)/td^3;

t1 = 0:0.001:td;
p1 = a01 + a11*t1 + a21*t1.^2 + a31*t1.^3;
v1 = a11 + 2*a21*t1 + 3*a31*t1.^2;
a1 = 2*a21 + 6*a31*t1;
p = p1;
v = v1;
a = a1;

h1v = thetag - thetav;
a02 = thetav;
a12 = vv;
a22 = (3*h1v-(2*vv+vg)*td)/td^2;
a32 = (-2*h1v+(vv+vg)*td)/td^3;

t2 = 0:0.001:td;
p2 = a02 + a12*t2 + a22*t2.^2 + a32*t2.^3;
v2 = a12 + 2*a22*t2 + 3*a32*t2.^2;
a2 = 2*a22 + 6*a32*t2;
p = [p p2];
v = [v v2];
a = [a a2];

degree2rad = pi/180;
p = [p*degree2rad;p*degree2rad];
v = [v*degree2rad;v*degree2rad];
a = [a*degree2rad;a*degree2rad];
end