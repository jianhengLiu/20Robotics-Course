function [angleZ1,angleY2,angleZ3] = exp2zyz(axis,theta)
% please input angle in rad
% angleZ1->alpha,angleY2->beta,angleZ3->gama
% number behind represents order
axis_h=[0,       -axis(3),axis(2);
        axis(3), 0,       -axis(1);
        -axis(2),axis(1), 0];
R=eye(3)+axis_h*sin(theta)+(axis_h^2)*(1-cos(theta))

angleY2 = atan2(sqrt(R(3,1)^2+R(3,2)^2),R(3,3));
angleZ1 = atan2(R(2,3)/sin(angleY2),R(1,3)/sin(angleY2));
angleZ3 = atan2(R(3,2)/sin(angleY2),-R(3,1)/sin(angleY2));

angleY2 = angleY2/pi*180;
angleZ1 = angleZ1/pi*180;
angleZ3 = angleZ3/pi*180;

if angleY2==0 || angleY2==180
    fprintf("Singularity!\n")
end
end