clear;
clc;
%建立机器人模型
% https://blog.csdn.net/u011280600/article/details/80448134
% https://www.bilibili.com/read/cv3807360
%       关节角 连杆偏距 连杆长度   连杆转角
%       theta    d        a        alpha     offset
 L1 = Link('d', 0, 'a', 0, 'alpha', 0,'modified');
 L2 = Link('d', 0, 'a', 1, 'alpha', 0,'modified');
 L3 = Link('theta', 0, 'a', 2, 'alpha', 0,'modified');
 L4 = Link('d', 0, 'a', 0, 'alpha', 0,'modified');
 bot = SerialLink([L1 L2 L3 L4]);%连接连杆
 bot.display();%显示D-H参数表
 forward_kinematics=bot.fkine([-0.2 0.1 10 0.1])%前向运动学
 bot.plot([pi/2 pi/2 pi/2 0],'workspace',[-5 5 -5 5 -5 5 ])