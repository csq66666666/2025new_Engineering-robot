% 忽略2个轴,将5轴看成3轴机械臂（参考哈工深），与8轴联合使用
%% 自定义控制器位姿
pitch = -30; yaw = 30; roll = 30; x = 100; y = 0; z = 0;

r11 = cosd(yaw)*sind(roll);
r12 = cosd(yaw)*cosd(roll);
r13 = -sind(yaw);
r21 = cosd(pitch)*sind(yaw)*sind(roll)-sind(pitch)*cosd(roll);
r22 = cosd(pitch)*sind(yaw)*cosd(roll)+sind(pitch)*sind(roll);
r23 = cosd(pitch)*cosd(yaw);
r31 = sind(pitch)*sind(yaw)*sind(roll)+cosd(pitch)*cosd(roll);
r32 = sind(pitch)*sind(yaw)*cosd(roll)-cosd(pitch)*sind(roll);
r33 = sind(pitch)*cosd(yaw);
px = x;
py = y + 397.44;
pz = z + 58;

T = [r11, r12, r13, px;
     r21, r22, r23, py;
     r31, r32, r33, pz;
       0,   0,   0,  1;]

%% 解算
angle = 180/pi;  %转化为弧度制
d1 = 58; d2 = 346; d3 = 51.44;
theta2 = asin(-r32)
theta1 = atan2(r22, r12)
theta3 = atan2(-r33, r31)
px - ((-cos(theta1) * sin(theta2) * sin(theta3) - sin(theta1) * cos(theta3)) * d3 - sin(theta1) * d2)
py - ((-sin(theta1) * sin(theta2) * sin(theta3) + cos(theta1) * cos(theta3)) * d3 + cos(theta1) * d2)
pz - (-cos(theta2) * sin(theta3) * d3 + d1)