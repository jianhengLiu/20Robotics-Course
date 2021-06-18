function [M,C] = compute_MC(theta,dtheta)
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
alpha = I1(3,3)+I2(3,3)+m1*r1^2+m2*(l1^2+r2^2);
beta = m2*l1*r2;
delta = I2(3,3)+m2*r2^2;

theta1 = theta(1,1);
theta2 = theta(2,1);
dtheta1 = dtheta(1,1);
dtheta2 = dtheta(2,1);

M = [alpha+2*beta*cos(theta2),delta+beta*cos(theta2);delta+beta*cos(theta2),delta];
C = [-beta*sin(theta2)*dtheta2,-beta*sin(theta2)*(dtheta1+dtheta2);beta*sin(theta2)*dtheta1,0];
end
