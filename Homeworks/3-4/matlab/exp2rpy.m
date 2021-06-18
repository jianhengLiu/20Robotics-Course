function [angleR1,angleP2,angleY3] = exp2rpy(axis,theta)
% please input angle in rad
% angleR1->Roll(X),angleP2->Pitch(Y),angleY3->Yaw(Z)
% number behind represents order
axis_h=[0,       -axis(3),axis(2);
        axis(3), 0,       -axis(1);
        -axis(2),axis(1), 0];
R=eye(3)+axis_h*sin(theta)+(axis_h^2)*(1-cos(theta))

angleP2 = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
angleY3 = atan2(R(2,1)/cos(angleP2),R(1,1)/cos(angleP2));
angleR1 = atan2(R(3,2)/cos(angleP2),R(3,3)/cos(angleP2));

angleP2 = angleP2/pi*180;
angleY3 = angleY3/pi*180;
angleR1 = angleR1/pi*180;

if angleP2==90 || angleP2==-90
    fprintf("Singularity!\n")
end
end