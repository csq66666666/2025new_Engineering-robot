%% 工程机器人八轴机械臂仿真
clear;
close all;
clc;
%% 参数定义
%角度转换
angle = pi/180;  %转化为弧度制
 
%D-H参数表
theta1 = 0;     d1 = 0;     a1 = 0;     alpha1 = 0;     jointtype1 = 1;
theta2 = -pi/2; d2 = 0;     a2 = 0;     alpha2 = -pi/2; jointtype2 = 1;
theta3 = pi/2;  d3 = 0;     a3 = 0;     alpha3 = pi/2;  jointtype3 = 1;
theta4 = -pi/2; d4 = 58;    a4 = 0;     alpha4 = pi/2;  jointtype4 = 0;
theta5 = -pi/2; d5 = 119;   a5 = 0;     alpha5 = -pi/2; jointtype5 = 0;
theta6 = pi/2;  d6 = 0;     a6 = 0;     alpha6 = pi/2;  jointtype6 = 0;
theta7 = -pi/2; d7 = 0;     a7 = 227;   alpha7 = 0;     jointtype7 = 0;
theta8 = 0;     d8 = 51.44;     a8 = 0;     alpha8 = -pi/2; jointtype8 = 0;

%% DH法建立模型,关节转角，关节距离，连杆长度，连杆转角，关节类型（0转动，1移动），'standard'：建立标准型D-H参数
L(1) = Link([theta1, d1, a1, alpha1, jointtype1], 'modified');
L(2) = Link([theta2, d2, a2, alpha2, jointtype2], 'modified');
L(3) = Link([theta3, d3, a3, alpha3, jointtype3], 'modified');
L(4) = Link([theta4, d4, a4, alpha4, jointtype4], 'modified');
L(5) = Link([theta5, d5, a5, alpha5, jointtype5], 'modified');
L(6) = Link([theta6, d6, a6, alpha6, jointtype6], 'modified');
L(7) = Link([theta7, d7, a7, alpha7, jointtype7], 'modified');
L(8) = Link([theta8, d8, a8, alpha8, jointtype8], 'modified');

% 定义关节中值
L(4).offset = -pi/2;
L(5).offset = -pi/2;
L(6).offset = pi/2;
L(7).offset = -pi/2;

% 定义关节范围
L(1).qlim =[0, 570];
L(2).qlim =[0, 300];
L(3).qlim =[0, 442];
L(4).qlim =[-180*angle, 180*angle];
L(5).qlim =[-120*angle, 120*angle];
L(6).qlim =[-20*angle, 90*angle];
L(7).qlim =[-90*angle, 90*angle];
L(8).qlim =[-180*angle, 180*angle];
 
%% 显示机械臂（把上述连杆“串起来”）
robot = SerialLink(L,'name','Engineering')
init = [0 0 0 0 0 0 0 0];   % 初始关节角度
figure(1)
robot.plot(init);
title('工程八轴机械臂模型');

%% 加入teach指令，则可调整各个关节角度
robot = SerialLink(L,'name','Engineering');
init = [0 0 0 0 0 0 0 0];   % 初始关节角度
figure(2)
robot.plot(init);
robot.teach
title('工程八轴机械臂模型可调节');

%% 求解运动学正解
robot = SerialLink(L,'name','Engineering');
theta = [0 0 0 0 0 0 0 0];
p = robot.fkine(theta)       			%fkine正解函数，根据关节角theta，求解出末端位姿p
% q = robot.ikine(p, 'mask',[1 1 1 1 1 0])            			%ikine逆解函数，根据末端位姿p，求解出关节角q

%% 1
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