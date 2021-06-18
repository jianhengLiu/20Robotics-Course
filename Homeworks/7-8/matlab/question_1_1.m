clear;
clc;
syms theta theta1 theta2 real;
syms lx ly lz l0x l0y l0z l1x l1y l1z l2x l2y l2z real;

% 初始位姿
gst0=[eye(3),[l0x;l0y;l0z];
        0,0,0,1];
% 期望位姿
gst=[rotz(theta),[lx;ly;lz];
        0,0,0,1]; 
% 方便运算的g1,expinv()为自己写的符号运算求逆
g1=gst*expinv(gst0);
% 设两z方向的平行转轴    
w1=[0;0;1];w2=w1;

q1=[0;0;0];
q2=[0;l1y;0];

v1 = -cross(w1,q1);
s1=[v1;w1];
v2 = -cross(w2,q2);
s2=[v2;w2];

expw1=angvec2r(theta1,w1);
expw2=angvec2r(theta2,w2);

exps1=[expw1,(eye(3)-expw1)*cross(w1,v1)+w1*w1'*v1*theta1;0,0,0,1];
exps2=[expw2,(eye(3)-expw2)*cross(w2,v2)+w2*w2'*v2*theta2;0,0,0,1];

% 取q不属于w1,w2转轴
% q1属于w1转轴
% r2属于w2转轴
syms qx qy qz q1z r2x r2y r2z real;
q=[qx;qy;qz];
q1=[0;0;q1z];
r2=[r2x;r2y;r2z];
u2=q-r2;
v2=q1-r2;
u2d=u2-w2*w2'*u2;
v2d=v2-w2*w2'*v2;
theta0=atan2(w2'*cross(u2d,v2d),u2d'*v2d);
deltad=(v2-expw2*u2)'*(v2-expw2*u2)-(w2'*(q-q2))'*(w2'*(q-q2));
Phi = acos((u2d'*u2d+v2d'*v2d-deltad)/(2*sqrt(u2d'*u2d)*sqrt(v2d'*v2d)))
theta21=theta0+Phi;
theta22=theta0-Phi;
% 解得theta2
% 更新一下相关
expw2=angvec2r(theta2,w2);
exps2=[expw2,(eye(3)-expw2)*cross(w2,v2)+w2*w2'*v2*theta2;0,0,0,1];

% r1属于w1转轴
syms qx qy qz q1z r1x r1y r1z real;
r1=[r1x;r1y;r1z];

q_ho=[q;1];
u1=exps2*q_ho-r1;
v1=g1*q_ho-r1;
u1d=u1-w1*w1'*u1;
v1d=v1-w1*w1'*v1;

theta1=atan2(w1'*cross(u1d,v1d),u1d'*v1d)
% 解得theta1