clear;
clc;
  syms l0 l1 l2;
%建立机器人模型
% https://blog.csdn.net/u011280600/article/details/80448134
% https://www.bilibili.com/read/cv3807360
%       关节角 连杆偏距 连杆长度   连杆转角
%       theta    d        a        alpha     offset
 L1 = Link('d', 1, 'a', 0, 'alpha', 0,'modified');
 L2 = Link('d', 0, 'a', 0, 'alpha', pi/2,'modified');
 L3 = Link('d', 0, 'a', 1, 'alpha', 0,'modified');
 L4 = Link('d', 0, 'a', 1, 'alpha', 0,'modified');
 L5 = Link('d', 1, 'a', 0, 'alpha', -pi/2,'modified');
 L6 = Link('d', 0, 'a', 1, 'alpha', pi/2,'modified');
 bot = SerialLink([L1 L2 L3 L4 L5 L6]);%连接连杆
 bot.display();%显示D-H参数表
%   syms theta1 theta2 theta3 theta4;
%  forward_kinematics=bot.fkine([theta1 theta2 theta4 theta3])%前向运动学
 bot.plot([0 0 0 0 0 0],'workspace',[-5 5 -5 5 -5 5 ])