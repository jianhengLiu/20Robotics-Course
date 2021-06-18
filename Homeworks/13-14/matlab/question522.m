degree2rad = pi/180;

theta0 = 5;
thetav = 15;
thetag = -10;

td = 2;

subplot(3,2,1)
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
a01 = theta0
a11 = v0
a21 = (3*h0v-(2*v0+vv)*td)/td^2
a31 = (-2*h0v+(v0+vv)*td)/td^3

t1 = 0:0.01:td;
p1 = a01 + a11*t1 + a21*t1.^2 + a31*t1.^3;
v1 = a11 + 2*a21*t1 + 3*a31*t1.^2;
a1 = 2*a21 + 6*a31*t1;
t = t1;
p = p1;
v = v1;
a = a1;

h1v = thetag - thetav;
a02 = thetav
a12 = vv
a22 = (3*h1v-(2*vv+vg)*td)/td^2
a32 = (-2*h1v+(vv+vg)*td)/td^3

t2 = 0:0.01:td;
p2 = a02 + a12*t2 + a22*t2.^2 + a32*t2.^3;
v2 = a12 + 2*a22*t2 + 3*a32*t2.^2;
a2 = 2*a22 + 6*a32*t2;
t = [t t2+t(end)];
p = [p p2];
v = [v v2];
a = [a a2];

subplot(3,2,1)
plot(t,p,'b')
title('position')
subplot(3,2,3)
plot(t,v,'b')
title('velocity')
subplot(3,2,5)
plot(t,a,'b')
title('acceleration')

% 机械臂参数
m1 = 12;
m2 = 12;

l1 = 1;
l2 = 1;

r1 = 0.5;
r2 = 0.5;

I1 = [0.0125,0,0;0,1.0025,0;0,0,1.01];
I2 = I1;

% 运动学方程
alpha = I1(3,3)+I2(3,3)+m1*r1^2+m2*(l1^2+r2^2)
beta = m2*l1*r2
delta = I2(3,3)+m2*r2^2

torque = zeros(2,length(t(:)));
for k=1:length(t(:))
    theta1 = p(1,k)*degree2rad;
    theta2 = p(1,k)*degree2rad;
    dtheta1 = v(1,k)*degree2rad;
    dtheta2 = v(1,k)*degree2rad;
    ddtheta1 = a(1,k)*degree2rad;
    ddtheta2 = a(1,k)*degree2rad;
    M = [alpha+2*beta*cos(theta2),delta+beta*cos(theta2);
        delta+beta*cos(theta2), delta];
    C = [-beta*sin(theta2)*dtheta2,-beta*sin(theta2)*(dtheta1+dtheta2);
        beta*sin(theta2)*dtheta1,0];
    torque(:,k) = M*[ddtheta1;ddtheta2]+C*[dtheta1;dtheta2];
end
subplot(3,2,2)
plot(t,torque(1,:));
title('Torque1');
subplot(3,2,4)
plot(t,torque(2,:));
title('Torque2');