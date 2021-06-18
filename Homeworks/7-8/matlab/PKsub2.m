function [theta2,theta1] = PKsub2(w2,w1,r,p,q)
%   exps1*exps2*p=q
%   w2第一转角的转轴 w1第二转角的转轴 r两转轴的交点 p初始点 q终止点

u=p-r;
v=q-r;
% 设z=c-r=αw1+βw2+Ω（w1xw2)
alpha=((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
beta=((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
gamma=(u'*u-alpha^2-beta^2-2*alpha*beta*w1'*w2)/(cross(w1,w2)'*cross(w1,w2));
z=alpha*w1+beta*w2+sqrt(gamma)*cross(w1,w2);

% c为中间过度点，两圆的交点
c=z+r;
% 转化为两个子问题1
% exp(s2h*theta2)*p=c
theta2 = PKsub1(w2,r,p,c);
% exp(-s1h*theta1)*q=c --> exp(s1h*theta1)*c=q
theta1= PKsub1(w1,r,c,q);
end

