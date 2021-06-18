clear;
clc;
syms theta1 alpha0 a0 d1 theta2 alpha1 a1 d2 theta3 alpha2 a2 d3 L2 ;
A1 = [cos(theta1),-sin(theta1),0,a0;
    sin(theta1)*cos(alpha0),cos(theta1)*cos(alpha0),-sin(alpha0),-sin(alpha0)*d1;
sin(theta1)*sin(alpha0),cos(theta1)*sin(alpha0),cos(alpha0),cos(alpha0)*d1;
0,0,0,1]
A2 =[cos(theta2),-sin(theta2),0,a1;
    sin(theta2)*cos(alpha1),cos(theta2)*cos(alpha1),-sin(alpha1),-sin(alpha1)*d2;
sin(theta2)*sin(alpha1),cos(theta2)*sin(alpha1),cos(alpha1),cos(alpha1)*d2;
0,0,0,1];
A3 =[cos(theta3),-sin(theta3),0,a2;
    sin(theta3)*cos(alpha2),cos(theta3)*cos(alpha2),-sin(alpha2),-sin(alpha2)*d3;
sin(theta3)*sin(alpha2),cos(theta3)*sin(alpha2),cos(alpha2),cos(alpha2)*d3;
0,0,0,1];
% alpha0 = 0;a0= 0;d1 = 0;theta1 =theta1;
% alpha1 = 90;a1= 0;d2 = 0;theta2 =theta2;
% alpha2 = 0;a2= 0;d3 = L2;theta3 =theta3;
alpha0 = sym(0);a0 =sym(0);theta1=sym(theta1);d1 = sym(0);
alpha1 =sym(pi/2);a1= sym(0);d2 = d2;theta2 =sym(theta2);
alpha2 = sym(0);a2= sym(0);d3 = L2;theta3 =sym(theta3);
T1 = simplify(eval(A1))
T=simplify(eval(A1*A2*A3))
