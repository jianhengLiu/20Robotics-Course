
syms theta1 alpha0 a0 d1 theta2 alpha1 a1 d2 theta3 alpha2 a2 d3 L2 ;
%建立机器人模型
%       关节角 连杆偏距 连杆长度   连杆转角
%       theta    d        a        alpha     offset
L1=Link([0       0        a0         0      0     ]); %定义连杆的D-H参数，注意参数的顺序
L2=Link([0       0.5      1.8       0      0     ]);
L3=Link([0       0        1         -pi/2      0     ]);
%  L4=Link([0       0        1         0          0     ]);
%  L5=Link([0       0        2    0     0     ]);
%  L6=Link([0       0        2    0     0     ]);
robot=SerialLink([L1 L2 L3  ],'name','manman') %连接连杆，机器人取名manman
robot.fkine([0 0 0])
% A=unifrnd(-pi,pi,[1,30000]);%第一关节变量限位
% B=unifrnd(-pi,pi,[1,30000]);%第二关节变量限位
% C=unifrnd(-pi,pi,[1,30000]);%第三关节变量限位
% D=unifrnd(-pi,pi,[1,30000]);%第三关节变量限位
% G= cell(30000, 3);%建立元胞数组
% for n = 1:30000 %for循环的执行次数直接决定了程序要跑多久
%     G{n} =[A(n) B(n) C(n) ];
% end                                         %产生3000组随机点
% H1=cell2mat(G);                       %将元胞数组转化为矩阵
% T=double(robot.fkine(H1));       %机械臂正解
% figure(1)
% scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)))%随机点图
robot.plot([0 0 0],'workspace',[-5 5 -5 5 -5 5 ],'tilesize',2)%机械臂图