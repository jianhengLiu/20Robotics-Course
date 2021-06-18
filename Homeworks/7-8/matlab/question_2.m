% 设r=（0，0，0）'
r=[0;0;0];
syms px py pz real;
p=[px;py;pz];
delta0=sqrt(p'*p);

% 通过三球面方程联立求得q点
syms x y z q1x q1y q1z delta1 q2x q2y q2z delta2 real;
eq1=(x-q1x)^2+(y-q1y)^2+(z-q1z)^2-delta1^2==0;
eq2=(x-q2y)^2+(y-q2y)^2+(z-q2z)^2-delta2^2==0;
% 设r点为（0，0，0）
eq3=x^2+y^2+z^2-delta0^2==0;
[x,y,z]=solve([eq1,eq2,eq3],[x,y,z],'Real',true);
q=[x,y,z];

% 子问题2
syms w1x w1y w1z w2x w2y w2z real;
w1=[w1x;w1y;w1z];
w2=[w2x;w2y;w2z];
u=p-r;
v=q-r
% 设z=c-r=αw1+βw2+Ω（w1xw2)
alpha=((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
beta=((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
gamma=(u'*u-alpha^2-beta^2-2*alpha*beta*w1'*w2)/(cross(w1,w2)'*cross(w1,w2));
z=alpha*w1+beta*w2+gamma*cross(w1,w2);

% 子问题1
% v=c-r=z
u2=p-r;
u1=q-r;
v1=z;v2=z;
u2d=u2-w2*w2'*u2;
u1d=u1-w1*w1'*u1;
v2d=v2-w2*w2'*v2;
v1d=v1-w1*w1'*v1;

theta2=atan2(w2'*cross(u2d,v2d),u2d'*v2d);
theta1=atan2(w1'*cross(u1d,v1d),u1d'*v1d);