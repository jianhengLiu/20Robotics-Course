clc
clear
% 初始化机械臂参数
theta = [0;0];
dtheta = [0;0];
ddtheta = [0;0];

K1=150;
K2=10;

T=6.501;

for t = 0:0.001:T
    k = int16(t*1000+1);
    
    theta_desired=[0.3+0.02*sin(t);0.1+0.01*cos(t)];
    dtheta_desired=[0.02*cos(t);-0.01*sin(t)];
    ddtheta_desired=[-0.02*sin(t);-0.01*cos(t)];
    theta_desired_plot(:,k)=theta_desired;
    dtheta_desired_plot(:,k)=dtheta_desired;
    ddtheta_desired_plot(:,k)=ddtheta_desired;
    
    %     计算误差
    theta_error = theta-theta_desired;
    dtheta_error = dtheta-dtheta_desired;
    theta_error_plot(:,k) = theta_error;
    dtheta_error_plot(:,k) = dtheta_error;
    
    % 获取M，C矩阵
    [M,C,g]=compute_MCg(theta,dtheta);
    
    torque=-K1*(theta_error)-K2*(dtheta_error)+M*ddtheta_desired+C*dtheta_desired+g;
    
    torque_plot(:,k) = torque;
    
    ddtheta=inv(M)*(torque-C*dtheta-g);
    
    ddtheta_plot(:,k) = ddtheta;
    dtheta = dtheta + ddtheta*0.001;
    dtheta_plot(:,k) = dtheta;
    theta = theta + dtheta*0.001 + ddtheta*0.001^2/2;
    theta_plot(:,k) = theta;
end
t = 0:0.001:T;
subplot(4,2,1)
plot(t,theta_plot(1,:),'r');
hold on
plot(t,theta_desired_plot(1,:),'b');
title('theta1')
legend('实际输出','期望输出');
subplot(4,2,3)
plot(t,dtheta_plot(1,:),'r');
hold on
plot(t,dtheta_desired_plot(1,:),'b');
legend('实际输出','期望输出');
title('dtheta1')
subplot(4,2,5)
plot(t,torque_plot(1,:),'r');
hold on
title('torque1')
legend('实际输出');
subplot(4,2,7)
plot(t,theta_error_plot(1,:),'b');
title('theta1 error')

subplot(4,2,2)
plot(t,theta_plot(2,:),'r');
hold on
plot(t,theta_desired_plot(2,:),'b');
title('theta2')
legend('实际输出','期望输出');
subplot(4,2,4)
plot(t,dtheta_plot(2,:),'r');
hold on
plot(t,dtheta_desired_plot(2,:),'b');
legend('实际输出','期望输出');
title('dtheta2')
subplot(4,2,6)
plot(t,torque_plot(2,:),'r');
hold on
title('torque2')
legend('实际输出');
subplot(4,2,8)
plot(t,theta_error_plot(2,:),'b');
title('theta2 error')