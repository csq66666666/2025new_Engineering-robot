%% 工程机器人滑移机构仿真
clear;
close all;
clc;
%% 参数定义
%角度转换
angle = pi/180;  %转化为弧度制

%D-H参数表
theta1 = 0;     D1 = 0; A1 = 0; alpha1 = 0;     offset1 = 1;
theta2 = -pi/2; D2 = 0; A2 = 0; alpha2 = -pi/2; offset2 = 1;
theta3 = pi/2;  D3 = 0; A3 = 0; alpha3 = pi/2;  offset3 = 1;

%% DH法建立模型,关节转角，关节距离，连杆长度，连杆转角，关节类型（0转动，1移动），'modified'：建立改进型D-H参数
L(1) = Link([theta1, D1, A1, alpha1, offset1], 'modified')
L(2) = Link([theta2, D2, A2, alpha2, offset2], 'modified')
L(3) = Link([theta3, D3, A3, alpha3, offset3], 'modified')

L(3).offset = 0

% 定义关节范围
L(1).qlim =[0, 4000];
L(2).qlim =[0, 4000];
L(3).qlim =[0, 4000];
 
%% 显示机械臂（把上述连杆“串起来”）
robot0 = SerialLink(L,'name','five');
init = [0 0 0];    %初始关节角度
figure(1)
robot0.plot(init);
title('工程机械臂模型');

%% 加入teach指令，则可调整各个关节角度
robot1 = SerialLink(L,'name','five');
figure(2)
robot1.plot(init);
robot1.teach
title('工程五轴机械臂模型可调节');
