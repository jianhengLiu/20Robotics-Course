clc
clear
% 初始化机械臂参数
theta = [deg2rad(5);deg2rad(5)];
dtheta = [0;0];
ddtheta = [0;0];

% 获取M，C矩阵
[M,C]=compute_MC(theta,dtheta);

% 获取期望轨迹
[theta_desired,dtheta_desired,ddtheta_desired] = trajectory_problem2();

for t = 0:0.001:4.001
    %     计算误差
    k = int16(t*1000+1);
    theta_error = theta_desired(:,k) - theta;
    dtheta_error = dtheta_desired(:,k) - dtheta;
    theta_error_plot(:,k) = theta_error;
    % 更新M，C矩阵
    [M,C]=compute_MC(theta,dtheta);
    [M_desired,C_desired]=compute_MC(theta_desired(:,k),dtheta_desired(:,k));
    %     更新期望速度
    dtheta_desired_t = dtheta_desired(:,k);
    %     更新期望加速度
    ddtheta_desired_t = ddtheta_desired(:,k);
    %   获取无饱和的期望力矩
    torque_desired(:,k) = M_desired*ddtheta_desired(:,k)+C_desired*dtheta_desired(:,k);
    % 获取AugmentedPD控制器的输出力矩
    Kp = 150;
    Kv = 10;
    torque = M*ddtheta_desired_t + C*dtheta_desired_t + Kp*theta_error + Kv*dtheta_error;
    %     饱和函数
    if abs(torque(1,1)) >= 20
        torque(1,1) = torque(1,1)/abs(torque(1,1))*20;
    end
    if abs(torque(2,1)) >= 20
        torque(2,1) = torque(2,1)/abs(torque(2,1))*20;
    end
    torque_plot(:,k) = torque;
    %     真实关节加速度
    ddtheta = inv(M)*(torque-C*dtheta);
    ddtheta_plot(:,k) = ddtheta;
    dtheta = dtheta + ddtheta*0.001;
    dtheta_plot(:,k) = dtheta;
    theta = theta + dtheta*0.001 + ddtheta*0.001^2/2;
    theta_plot(:,k) = theta;
end
t = 0:0.001:4.001;
subplot(4,2,1)
plot(t,theta_plot(1,:),'r');
hold on
plot(t,theta_desired(1,:),'b');
title('theta1')
legend('实际输出','期望输出');
subplot(4,2,3)
plot(t,dtheta_plot(1,:),'r');
hold on
plot(t,dtheta_desired(1,:),'b');
legend('实际输出','期望输出');
title('dtheta1')
subplot(4,2,5)
plot(t,torque_plot(1,:),'r');
hold on
plot(t,torque_desired(1,:),'b');
title('torque1')
legend('实际输出','期望输出');
subplot(4,2,7)
plot(t,theta_error_plot(1,:),'b');
title('theta_error1')

subplot(4,2,2)
plot(t,theta_plot(2,:),'r');
hold on
plot(t,theta_desired(2,:),'b');
title('theta2')
legend('实际输出','期望输出');
subplot(4,2,4)
plot(t,dtheta_plot(2,:),'r');
hold on
plot(t,dtheta_desired(2,:),'b');
legend('实际输出','期望输出');
title('dtheta2')
subplot(4,2,6)
plot(t,torque_plot(2,:),'r');
hold on
plot(t,torque_desired(2,:),'b');
title('torque2')
legend('实际输出','期望输出');
subplot(4,2,8)
plot(t,theta_error_plot(2,:),'b');
title('theta_error2')