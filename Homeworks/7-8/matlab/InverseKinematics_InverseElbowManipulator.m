function [theta1,theta2,theta3,theta4,theta5,theta6] = InverseKinematics_InverseElbowManipulator(gst0,gst_target,l0,l1,l2)
%	gst0:初始位姿 gst_target:目标位姿

w1=[0;0;1];
w2=[0;1;0];w6=w2;
w3=[-1;0;0];w4=w3;w5=w3;

q1=[0;0;0];
q2=[0;0;l0];
q3=[0;0;l0];
q4=[0;l1;l0];
q5=[0;l1+l2;l0];
q6=q5;

g1=gst_target*inv(gst0);
p5=expsMultiplyVector(g1,q5);
theta1=atan2(-p5(1,1),p5(2,1));

% exps为自己写的转化为指数形式的函数
exps1=exps(w1,q1,theta1);
p51=expsMultiplyVector(expinv(exps1),p5);
theta2=atan2(p51(1,1),p51(3,1)-1);

exps2=exps(w2,q2,theta2);
p512=expsMultiplyVector(expinv(exps2),p51);
 [theta4,theta3] = PKsub2_non_intersecting(w4,w3,q4,q3,q5,p512);
 
exps3=exps(w3,q3,theta3);
exps4=exps(w4,q4,theta4);
%  p不属于w5和w6
p=[1;0;0];
q=expsMultiplyVector(expinv(exps4)*expinv(exps3)*expinv(exps2)*expinv(exps1)*g1,p);
 [theta6,theta5] = PKsub2(w6,w5,p5,p,q);
end

