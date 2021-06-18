function [axis,theta] = rpy2exp(angleR1,angleP2,angleY3)
% please input angle in rad
% R->X, P->Y, Y->Z
% number behind represents order
RR1 = [1,            0,            0;
       0,            cos(angleR1), -sin(angleR1);
       0,            sin(angleR1), cos(angleR1)];
RY2 = [cos(angleP2), 0,            sin(angleP2);
       0,            1,            0;
       -sin(angleP2),0,            cos(angleP2)];
RY3 = [cos(angleY3),-sin(angleY3),0;
       sin(angleY3),cos(angleY3), 0;
       0,           0,            1];
R = RY3*RY2*RR1
trR = R(1,1)+R(2,2)+R(3,3);

% acos->arccos
theta = acos((trR-1)/2);
axis = [R(3,2)-R(2,3);
        R(1,3)-R(3,1);
        R(2,1)-R(1,2)];
axis = axis/(2*sin(theta));
theta= theta/pi*180;

if trR == 3
    fprintf("R=I\n");
elseif trR==-1
    fprintf("Singularity!\n")
end
end