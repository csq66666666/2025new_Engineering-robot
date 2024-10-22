%% 工程机器人八轴机械臂仿真
clear;
close all;
clc;
%% 参数定义
%角度转换
angle = pi/180;  %转化为弧度制
 
%D-H参数表
theta1 = 0;     d1 = 58;    a1 = 0;   alpha1 = 0;       jointtype1 = 0;
theta2 = -pi/2; d2 = 346;   a2 = 0;   alpha2 = -pi/2;   jointtype2 = 0;
theta3 = 0;     d3 = 0;     a3 = 0;   alpha3 = pi/2;    jointtype3 = 0;
theta4 = 0;     d4 = 51.44;     a4 = 0;   alpha4 = -pi/2;   jointtype4 = 0;

%% DH法建立模型,关节转角，关节距离，连杆长度，连杆转角，关节类型（0转动，1移动），'standard'：建立标准型D-H参数
L(1) = Link([theta1, d1, a1, alpha1, jointtype1], 'modified');
L(2) = Link([theta2, d2, a2, alpha2, jointtype2], 'modified');
L(3) = Link([theta3, d3, a3, alpha3, jointtype3], 'modified');
L(4) = Link([theta4, d4, a4, alpha4, jointtype4], 'modified');

% 定义关节中值
L(2).offset = -pi/2;

% 定义关节范围
L(1).qlim =[-180*angle, 180*angle];
L(2).qlim =[-120*angle, 120*angle];
L(3).qlim =[-90*angle, 90*angle];
 
%% 显示机械臂（把上述连杆“串起来”）
robot = SerialLink(L,'name','Engineering_3DOF')
init = [0 0 0 0];   % 初始关节角度
figure(1)
robot.plot(init);
title('工程八轴机械臂模型');

%% 加入teach指令，则可调整各个关节角度
robot = SerialLink(L,'name','Engineering_3DOF');
init = [0 0 0 0];   % 初始关节角度
figure(2)
robot.plot(init);
robot.teach
title('工程八轴机械臂模型可调节');

%% 求解运动学正解
robot = SerialLink(L,'name','Engineering');
theta = [0 0 0 0 0 0 0 0];
p = robot.fkine(theta)       			%fkine正解函数，根据关节角theta，求解出末端位姿p
% q = robot.ikine(p, 'mask',[1 1 1 1 1 0])            			%ikine逆解函数，根据末端位姿p，求解出关节角q

%% 计算工作空间
N=30000;
D1 = 200*rand(N,1);
D2 = 200*rand(N,1);
D3 = 200*rand(N,1);
THETA4 = -pi    + 2*pi*rand(N,1);
THETA5 = -pi/2  + pi*rand(N,1);
THETA6 = -pi/2  + pi*rand(N,1);
THETA7 = -pi/2  + pi*rand(N,1);
THETA8 = -pi    + 2*pi*rand(N,1);

for n=1:1:30000
    pp=robot.fkine([D1(n) D2(n) D3(n) THETA4(n) THETA5(n) THETA6(n) THETA7(n) THETA8(n)]);
    plot3(pp.t(1),pp.t(2),pp.t(3),'b.','MarkerSize',0.5);
    hold on
end