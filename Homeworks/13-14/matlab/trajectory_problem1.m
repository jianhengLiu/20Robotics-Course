function [p,v,a] = trajectory_problem1()
theta1 = 5;
theta2 = 15;
theta3 = 40;

acceleration_b = 80;

td12 = 1; 
td23 = 1;

% subplot(3,1,1)
% plot(0,theta1,'r*');
% hold on;
% plot(td12,theta2,'r*');
% hold on;
% plot(td12+td23,theta3,'r*');
% hold on;

acceleration_1 = acceleration_b;
tb1 = td12 - sqrt(td12^2-2*(theta2-theta1)/acceleration_1);
velocity12 = (theta2-theta1)/(td12-tb1/2);

acceleration_3 = -acceleration_b;
tb3 = td23 - sqrt(td23^2 + 2*(theta3-theta2)/acceleration_3);
velocity23 = (theta3-theta2)/(td23-tb3/2);

acceleration_2 = acceleration_b;
tb2 = (velocity23-velocity12)/acceleration_2;

t23 = td23 - tb2/2 - tb3;

t12 = td12 - tb2/2 - tb1;

t_b1 = 0:0.001:tb1;
p_b1 = theta1 + 1/2*acceleration_1*t_b1.^2;
v_b1 = acceleration_1*t_b1;
a_b1 = acceleration_1 + 0*t_b1;
p = p_b1;
v = v_b1;
a = a_b1;

t_12 = 0:0.001:t12;
p_12 = p_b1(end) + velocity12.*t_12;
v_12 = v_b1(end) + 0 * t_12;
a_12 = 0*t_12;
p = [p p_12];
v = [v v_12];
a = [a a_12];

t_b2 = 0:0.001:tb2;
p_b2 = p_12(end) + v_12(end)*t_b2 + 1/2*acceleration_2*t_b2.^2;
v_b2 = v_12(end) + acceleration_2*t_b2;
a_b2 = acceleration_2 +0*t_b2;
p = [p p_b2];
v = [v v_b2];
a = [a a_b2];

t_23 = 0:0.001:t23;
p_23 = p_b2(end) + velocity23*t_23;
v_23 = v_b2(end) + 0*t_23;
a_23 = 0*t_23;
p = [p p_23];
v = [v v_23];
a = [a a_23];

t_b3 = 0:0.001:tb3;
p_b3 = p_23(end) + v_23(end)*t_b3 + 1/2*acceleration_3*t_b3.^2;
v_b3 = v_23(end) + acceleration_3*t_b3;
a_b3 = acceleration_3 + 0*t_b3;
p = [p p_b3];
v = [v v_b3];
a = [a a_b3];

degree2rad = pi/180;
p = [p*degree2rad;p*degree2rad];
v = [v*degree2rad;v*degree2rad];
a = [a*degree2rad;a*degree2rad];
end