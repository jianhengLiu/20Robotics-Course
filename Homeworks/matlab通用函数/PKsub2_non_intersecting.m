function [theta2,theta1] = PKsub2_non_intersecting(w2,w1,q2,q1,p,q)
%   exps1*exps2*p=q
%   w2:第一转角的转轴 w1:第二转角的转轴 q2:转轴w2上一点 q1:转轴w1上一点 p:初始点 q:终止点

u2=p-q2;
v2=q1-q2;
u2d=u2-w2*w2'*u2;
v2d=v2-w2*w2'*v2;
u2d_abs = sqrt(u2d'*u2d);
v2d_abs = sqrt(v2d'*v2d);

delta = sqrt((q-q1)'*(q-q1));
deltad_square = delta^2-(w2'*(p-q1))'*(w2'*(p-q1));

theta0=atan2(w2'*cross(u2d,v2d),u2d'*v2d);
Phi = acos((u2d_abs^2+v2d_abs^2-deltad_square)/(2*u2d_abs*v2d_abs));
theta2=theta0-Phi;

% exps为自己写的转化为指数形式的函数
exps2=exps(w2,q2,theta2);

p_ho=[p;1];
p1=mat2cell(exps2*p_ho,[3,1],[1]);
u1=p1{1,1}-q1;
v1=q-q1;
u1d=u1-w1*w1'*u1;
v1d=v1-w1*w1'*v1;
theta1=atan2(w1'*cross(u1d,v1d),u1d'*v1d);

end

