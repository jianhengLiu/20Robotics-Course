clear;
clc;
syms theta1 theta2 theta3 theta4 theta5 theta6;
syms l0 l1 l2 l3 l4 l5 l6 l7;

gst0=[eye(3),[0;l3+l4+l5;l1];
        0,0,0,1];

w1=[0;0;1];w5=w1;
w2=[1;0;0];w3=w2;w4=w2;
w6=[0;1;0];

q1=[0;0;0];
q2=[0;0;l0];
q3=[0;l2;l0];
q4=[0;l2+l3;l0];
q5=[-l4;l2+l3-l5;0];
q6=[-l4;l2+l3-l5+l6;l0+l1-l7];

v1 = -cross(w1,q1);
s1=[v1;w1];
v2 = -cross(w2,q2);
s2=[v2;w2];
v3 = -cross(w3,q3);
s3=[v3;w3];
v4 = -cross(w4,q4);
s4=[v4;w4];
v5 = -cross(w5,q5);
s5=[v5;w5];
v6 = -cross(w6,q6);
s6=[v6;w6];

expw1=angvec2r(theta1,w1);
expw2=angvec2r(theta2,w2);
expw3=angvec2r(theta3,w3);
expw4=angvec2r(theta4,w4);
expw5=angvec2r(theta5,w5);
expw6=angvec2r(theta6,w6);


exps1=[expw1,(eye(3)-expw1)*cross(w1,v1)+w1*w1'*v1*theta1;0,0,0,1];
exps2=[expw2,(eye(3)-expw2)*cross(w2,v2)+w2*w2'*v2*theta2;0,0,0,1];
exps3=[expw3,(eye(3)-expw3)*cross(w3,v3)+w3*w3'*v3*theta3;0,0,0,1];
exps4=[expw4,(eye(3)-expw4)*cross(w4,v4)+w4*w4'*v4*theta4;0,0,0,1];
exps5=[expw5,(eye(3)-expw5)*cross(w5,v5)+w5*w5'*v5*theta5;0,0,0,1];
exps6=[expw6,(eye(3)-expw6)*cross(w6,v6)+w6*w6'*v6*theta6;0,0,0,1];

gst_theta=exps1*exps2*exps3*exps4*exps5*exps6*gst0;
gst_theta=simplify(gst_theta)