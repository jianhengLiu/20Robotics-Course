function [M,C,g] = compute_MCg(theta,dtheta)
m1=1;
m2=1.5;
l1=0.2;
l2=0.3;
lc1=0.1;
lc2=0.15;
J1=0.013;
J2=0.045;

Theta1=m1*lc1*lc1+m2*(l1*l1+lc2*lc2)+J1+J2;
Theta2=m2*l1*lc2;
Theta3=m2*lc2*lc2+J2;
Theta4=m1*lc1+m2*l1;
Theta5=m2*lc2;

gravity=9.8;

theta1 = theta(1,1);
theta2 = theta(2,1);
dtheta1 = dtheta(1,1);
dtheta2 = dtheta(2,1);

M=[Theta1+2*Theta2*cos(theta2),Theta3+Theta2*cos(theta2);
    Theta3+Theta2*cos(theta2),Theta3];
C=[-Theta2*sin(theta2)*dtheta2,-Theta2*sin(theta2)*(dtheta2+dtheta1);
    Theta2*sin(theta2)*dtheta1,0];
g=[Theta4*gravity*cos(theta1)+Theta5*gravity*cos(theta1+theta2);
    Theta5*gravity*cos(theta1+theta2)];
end
