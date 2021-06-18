clear;
clc;
syms theta1 theta2 Dtheta1 Dtheta2;
syms l0 l1 l2 l3 l4 l5 l6 l7;
R03=[cos(theta1+theta2),-sin(theta1+theta2),0;sin(theta1+theta2),cos(theta1+theta2),0;0,0,1];
R03t=[cos(theta1+theta2),sin(theta1+theta2),0;-sin(theta1+theta2),cos(theta1+theta2),0;0,0,1];

p03=[-sin(theta1+theta2)*l2-l1*sin(theta1);l2*cos(theta1+theta2)+l1*cos(theta1);l0+l3]
p03_h=[0,-p03(3),p03(2);p03(3),0,p03(1);p03(2),p03(1),0]
Ai=[R03t,-R03t*p03_h;zeros(3,3),R03t];
Ai=simplify(Ai);
V03s=[l1*cos(theta1)*Dtheta2+l2*(Dtheta1+Dtheta2);l1*sin(theta1)*Dtheta2;0;0;0;Dtheta1+Dtheta2];
V03b=Ai*V03s;
V03b=simplify(V03b)