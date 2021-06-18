function [theta1,theta2,theta3,theta4,theta5,theta6] = InverseKinematics_StanfordManipulator(gst0,gst_target,l0,l1,l2)
%	gst0:初始位姿 gst_target:目标位姿

w1=[0;0;1];w5=w1;
w2=[-1;0;0];w4=w2;
w6=[0;1;0];
w3=[0;0;0];

q1=[0;0;0];
q2=[0;0;l0];
q4=[0;l1;l0];
q5=[0;l1;0];
q6=q2;

g1=gst_target*inv(gst0);

theta3 = sqrt((l0-gst_target(3,4))^2+(gst_target(1,4)^2+gst_target(2,4)^2))-l1

p3=[q4(1);q4(2)+theta3;q4(3)];
p4=expsMultiplyVector(g1,q4);
[theta2,theta1] = PKsub2(w2,w1,q2,p3,p4);

exps3=[eye(3),[0;theta3;0];0,0,0,1];
exps2=exps(w2,q2,theta2);
exps1=exps(w1,q1,theta1);
p2=expsMultiplyVector(expinv(exps3)*expinv(exps2)*expinv(exps1)*g1,q2);
[theta5,theta4] = PKsub2(w5,w4,q4,q2,p2);


exps5=exps(w5,q5,theta5);
exps4=exps(w4,q4,theta4);
p=[1;0;0];
q=expsMultiplyVector(expinv(exps5)*expinv(exps4)*expinv(exps3)*expinv(exps2)*expinv(exps1)*g1,p);
[theta6] = PKsub1(w6,q6,p,q);
end

