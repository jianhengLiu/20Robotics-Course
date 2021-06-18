function [axis,theta] = zyz2exp(angleZ1,angleY2,angleZ3)
% please input angle in rad
% number behind represents order
RZ1 = [cos(angleZ1), -sin(angleZ1),0;
       sin(angleZ1), cos(angleZ1), 0;
       0,            0,            1];
RY2 = [cos(angleY2), 0,            sin(angleY2);
       0,            1,            0;
       -sin(angleY2),0,            cos(angleY2)];
RZ3 = [cos(angleZ3),-sin(angleZ3),0;
       sin(angleZ3),cos(angleZ3), 0;
       0,           0,            1];
R = RZ1*RY2*RZ3
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