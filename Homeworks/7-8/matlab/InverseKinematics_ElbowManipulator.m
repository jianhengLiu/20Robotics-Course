function [theta1,theta2,theta3,theta4,theta5,theta6] = InverseKinematics_ElbowManipulator(gst0,gst_target,l0,l1,l2)
%	gst0:初始位姿 gst_target:目标位姿

w1=[0;0;1];w5=w1;
w2=[-1;0;0];w3=w2;w4=w2;
w6=[0;1;0];

q1=[0;0;0];
q2=[0;0;l0];
q3=[0;l1;l0];
q4=[0;l1+l2;l0];
q5=[0;l1+l2;0];
q6=q2;
pb=q2;
pw=q4;
pw_ho=[pw;1];

g1=gst_target*inv(gst0);
p1_ho=mat2cell(g1*pw_ho,[3,1],[1]);
p1=p1_ho{1,1};
theta3 = PKsub3(w3,q3,pw,pb,sqrt((p1-pb)'*(p1-pb)));

% exps为自己写的转化为指数形式的函数
exps3=exps(w3,q3,theta3);

p3_ho=mat2cell(exps3*pw_ho,[3,1],[1]);
p3=p3_ho{1,1};
[theta2,theta1] = PKsub2(w2,w1,pb,p3,p1);

exps2=exps(w2,q2,theta2);
exps1=exps(w1,q1,theta1);
% expinv为自己写的指数形式求逆函数
g2=expinv(exps3)*expinv(exps2)*expinv(exps1)*g1;
pb_ho=[pb;1];
p2_ho=mat2cell(g2*pb_ho,[3,1],[1]);
p2=p2_ho{1,1};
[theta5,theta4] = PKsub2(w5,w4,pw,pb,p2);

exps5=exps(w5,q5,theta5);
exps4=exps(w4,q4,theta4);

% p为不在w6上的一点
p=[1;0;0];
p_ho=[p;1];
p6_ho=mat2cell(expinv(exps5)*expinv(exps4)*g2*p_ho,[3,1],[1]);
p6=p6_ho{1,1};
[theta6] = PKsub1(w6,q6,p,p6);
end

