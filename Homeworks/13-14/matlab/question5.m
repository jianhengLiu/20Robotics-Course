m1 = 12;
m2 = 12;

l1 = 1;
l2 = 1;

r1 = 0.5;
r2 = 0.5;

I1 = [0.0125,0,0;0,1.0025,0;0,0,1.01];
I2 = I1;

syms theta1 dtheta1 theta2 dtheta2 real
w1 = [0;0;dtheta1];
w2 = [0;0;dtheta1+dtheta2];
v1 = [-r1*sin(theta1)*dtheta1;r1*cos(theta1)*dtheta1;0];
v2 = [-l1*sin(theta1)*dtheta1-r2*sin(theta1+theta2)*(dtheta1+dtheta2);
      l1*cos(theta1)*dtheta1+r2*cos(theta1+theta2)*(dtheta1+dtheta2);
      0];
  
T = 1/2*m1*(v1'*v1)+1/2*w1'*I1*w1+1/2*m2*(v2'*v2)+1/2*w2'*I2*w2;
T = simplify(T)

alpha = I1(3,3)+I2(3,3)+m1*r1^2+m2*(l1^2+r2^2)
beta = m2*l1*r2
delta = I2(3,3)+m2*r2^2
M = [alpha+2*beta*cos(theta2),delta+beta*cos(theta2);
    delta+beta*cos(theta2), delta]

simplify(T - 1/2*[dtheta1,dtheta2]*M*[dtheta1;dtheta2])
simplify([-beta*sin(theta2)*dtheta2,-beta*sin(theta2)*(dtheta1+dtheta2)]*[dtheta1;dtheta2])

%   求逆解
%	gst0:初始位姿 gst_target:目标位姿
g1=gst_target*inv(gst0);

q=expsMultiplyVector(g1,q2);
[theta1] = PKsub1(w1,q1,q2,q);

exps1=exps(w1,q1,theta1);
q=expsMultiplyVector(expinv(exps1)*g1,q1);
[theta2] = PKsub1(w1,q2,q1,q);
exps2=exps(w2,q2,theta2);