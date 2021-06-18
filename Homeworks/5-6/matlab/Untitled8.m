clear;
clc;
%           关节角     连杆偏距 连杆长度   连杆转角
%           theta        d        a        alpha     
 L1 = Link('theta', 0,    'a', 0, 'alpha', -pi/2,'sym');%定义连杆
 L2 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'sym');
 L3 = Link( 'd', 0,    'a', 0, 'alpha', 0,    'sym');
 L4 = Link( 'd', 0,    'a', 0, 'alpha', -pi/2,'sym');
 L5 = Link( 'd', 0,    'a', 0, 'alpha', pi/2,'sym');
 L6 = Link('d', 0, 'a', 0, 'alpha', 0,    'sym');
 bot = SerialLink([L1 L2 L3 L4 L5 L6]);%连接连杆
 syms theta1 theta2 d3 theta4 theta5 theta6;
 forward_kinematics=bot.fkine([theta1 theta2 d3 theta4 theta5 theta6])%前向运动学
 bot.plot([0 0 0 0 0 0],'workspace')%机械臂图