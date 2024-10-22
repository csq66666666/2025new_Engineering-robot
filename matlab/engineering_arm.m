%% 工程机器人五轴机械臂仿真
clear;
close all;
clc;
%% 参数定义
%角度转换
angle = pi/180;  %转化为弧度制
 
%D-H参数表
theta1 = 0;     D1 = 58;    A1 = 0;   alpha1 = 0;       offset1 = 0;
theta2 = -pi/2; D2 = 119;   A2 = 0;   alpha2 = -pi/2;   offset2 = 0;
theta3 = pi/2;  D3 = 0;     A3 = 0;   alpha3 = pi/2;    offset3 = 0;
theta4 = -pi/2; D4 = 0;     A4 = 227; alpha4 = 0;       offset4 = 0;
theta5 = 0;     D5 = 51.44;     A5 = 0;   alpha5 = -pi/2;   offset5 = 0;

%% DH法建立模型,关节转角，关节距离，连杆长度，连杆转角，关节类型（0转动，1移动），'standard'：建立标准型D-H参数
L(1) = Link([theta1, D1, A1, alpha1, offset1], 'modified');
L(2) = Link([theta2, D2, A2, alpha2, offset2], 'modified');
L(3) = Link([theta3, D3, A3, alpha3, offset3], 'modified');
L(4) = Link([theta4, D4, A4, alpha4, offset4], 'modified');
L(5) = Link([theta5, D5, A5, alpha5, offset5], 'modified');

L(2).offset = -pi/2;
L(3).offset = pi/2;
L(4).offset = -pi/2;

% 定义关节范围
L(1).qlim =[-180*angle, 180*angle];
L(2).qlim =[-90*angle, 90*angle];
L(3).qlim =[-90*angle, 90*angle];
L(4).qlim =[-90*angle, 90*angle];
L(5).qlim =[-180*angle, 180*angle];
 
%% 显示机械臂（把上述连杆“串起来”）
robot = SerialLink(L,'name','five');
theta = [pi/3 0 pi/4 pi/3 pi/6];    %初始关节角度
figure(1)
robot.plot(theta)
title('工程机械臂模型');

%% 加入teach指令，则可调整各个关节角度
robot = SerialLink(L,'name','five');
theta = [0 0 0 0 0];    %初始关节角度
figure(2)
robot.plot(theta);
robot.teach
title('工程五轴机械臂模型可调节');

%% 求解运动学正解
robot = SerialLink(L,'name','five');
theta = [pi/6 pi/3 pi/6 -0 pi/10];   			%实验二指定的关节角
p = robot.fkine(theta)       			%fkine正解函数，根据关节角theta，求解出末端位姿p
% q = robot.ikine(p, 'mask',[1 1 1 1 1 0])            			%ikine逆解函数，根据末端位姿p，求解出关节角q

%% 工作空间
N=5000;                                              
theta1=-pi+2*pi*rand(N,1);
theta2=pi/4*rand(N,1);
theta3=-pi/2+pi*rand(N,1);
theta4=-pi+2*pi*rand(N,1);
theta5=-pi+2*pi*rand(N,1);
 
for n=1:1:5000
    pp=robot.fkine([theta1(n) theta2(n) theta3(n) theta4(n) theta5(n)]);
    plot3(pp.t(1),pp.t(2),pp.t(3),'b.','MarkerSize',5);
    hold on
end