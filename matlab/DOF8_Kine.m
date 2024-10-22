% 5轴机械臂+3轴滑移组成8轴
%% 8轴正运动学
L4 = 58; L5 = 119;  L6 = 227; L7 = 51.44;
d1 = 0; d2 = 0; d3 = 0;
theta4 = 0; theta5 = 0; theta6 = 40; theta7 = 0; theta8 = 0;

r11 =  cos(theta4) * sin(theta5) * cos(theta6 + theta7) * cos(theta8) + cos(theta4) * cos(theta5) * sin(theta8) - sin(theta4) * sin(theta6 + theta7) * cos(theta8);
r12 = -cos(theta4) * sin(theta5) * cos(theta6 + theta7) * sin(theta8) + cos(theta4) * cos(theta5) * cos(theta8) + sin(theta4) * sin(theta6 + theta7) * sin(theta8);
r13 = -cos(theta4) * sin(theta5) * sin(theta6 + theta7) - sin(theta4) * cos(theta6 + theta7);
r21 =  sin(theta4) * sin(theta5) * cos(theta6 + theta7) * cos(theta8) + sin(theta4) * cos(theta5) * sin(theta8) + cos(theta4) * sin(theta6 + theta7) * cos(theta8);
r22 = -sin(theta4) * sin(theta5) * cos(theta6 + theta7) * sin(theta8) + sin(theta4) * cos(theta5) * cos(theta8) - cos(theta4) * sin(theta6 + theta7) * sin(theta8);
r23 = -sin(theta4) * sin(theta5) * sin(theta6 + theta7) + cos(theta4) * cos(theta6 + theta7);
r31 =  cos(theta5) * cos(theta6 + theta7) * cos(theta8) - sin(theta5) * sin(theta8);
r32 = -cos(theta5) * cos(theta6 + theta7) * sin(theta8) - sin(theta5) * cos(theta8);
r33 =  -cos(theta5) * sin(theta6 + theta7);
px  = -d3 - cos(theta4) * sin(theta5) * sin(theta6) * L6 - sin(theta4) * cos(theta6) * L6 - sin(theta4) * L5;
py  =  d2 - sin(theta4) * sin(theta5) * sin(theta6) * L6 + cos(theta4) * cos(theta6) * L6 + cos(theta4) * L5;
pz  =  d1 - cos(theta5) * sin(theta6) * L6 + L4;

px = L7 * r13 + px;
py = L7 * r23 + py;
pz = L7 * r33 + pz;

T = [r11, r12, r13, px;
     r21, r22, r23, py;
     r31, r32, r33, pz;
       0,   0,   0,  1;]
% T = [r11, r12, r13, 0;
%      r21, r22, r23, 0;
%      r31, r32, r33, 0;
%        0,   0,   0, 1;]

%% 自定义控制器位姿
pitch = 30; yaw = 45; roll = 60; x = 0; y = 0; z = 0;

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
py = y;
pz = z;
py = y + 397.44;
pz = z + 58;

T = [r11, r12, r13, px;
     r21, r22, r23, py;
     r31, r32, r33, pz;
       0,   0,   0,  1;]
%% 8轴逆运动学 姿态相等 正式使用
d1 = 0; d2 = 0; d3 = 0;
solution_count = 0;
% tehta6
A = px - r13 * L7;  B = py - r23 * L7;  C = pz - L4 - r33 * L7;
cos_theta6 = (A^2 + B^2 + C^2 - L5^2 - L6^2) / (2 * L5 * L6);
if cos_theta6 > 1
    cos_theta6 = 1;
end
for i = 1 : 2
    if (abs(1 - abs(cos_theta6)) < 1e-10)
        sin_theta6 = 0;
    else
        if i == 1
            sin_theta6 = sqrt(1 - cos_theta6^2);
        else
            sin_theta6 = -sqrt(1 - cos_theta6^2);
        end
    end
    theta6 = atan2(sin_theta6, cos_theta6);

    % theta4
    rho = sqrt(A^2 + B^2);
    phi = atan2(A, B);
    cos_theta4_a_phi = (cos_theta6 * L6 + L5) / rho;
    for j = 1 : 2
        if (abs(1 - abs(cos_theta4_a_phi)) < 1e-10)
            sin_theta4_a_phi = 0;
        else
            if j == 1
                sin_theta4_a_phi = sqrt(1 - cos_theta4_a_phi^2);
            else
                sin_theta4_a_phi = -sqrt(1 - cos_theta4_a_phi^2);
            end
        end
        theta4_a_phi = atan2(sin_theta4_a_phi, cos_theta4_a_phi);
        theta4 = theta4_a_phi - phi;
        sin_theta4 = sin(theta4);
        cos_theta4 = cos(theta4);

        % theta7
        cos_theta6_a_theta7 =  -sin_theta4 * r13 + cos_theta4 * r23;
        for k = 1 : 2
            if (abs(1 - abs(cos_theta6_a_theta7)) < 1e-10)
                sin_theta6_a_theta7 = 0;
            else
                if k == 1
                    sin_theta6_a_theta7 = sqrt(1 - cos_theta6_a_theta7^2);
                else
                    sin_theta6_a_theta7 = -sqrt(1 - cos_theta6_a_theta7^2);
                end
            end
            theta6_a_theta7 = atan2(sin_theta6_a_theta7, cos_theta6_a_theta7);
            theta7 = theta6_a_theta7 - theta6;

            % theta5
            if (abs(sin_theta6_a_theta7) < 1e-10)
                theta5 = theta5;
                sin_theta5 = sin(theta5);
                cos_theta5 = cos(theta5);
            else
                sin_theta5 = (cos_theta4 * r13 + sin_theta4 * r23) / (-sin_theta6_a_theta7);
                cos_theta5 = r33 / (-sin_theta6_a_theta7);
                theta5 = atan2(sin_theta5, cos_theta5);
            end
            

            % theta8
            sin_theta8 = r11 * cos(theta4) * cos_theta5 + r21 * sin(theta4) * cos_theta5 - r31 * sin_theta5;
            cos_theta8 = r12 * cos(theta4) * cos_theta5 + r22 * sin(theta4) * cos_theta5 - r32 * sin_theta5;
            theta8 = atan2(sin_theta8, cos_theta8);

            solution = [d1 d2 d3 theta4 theta5 theta6 theta7 theta8]

            r11_ =  cos(theta4) * sin(theta5) * cos(theta6 + theta7) * cos(theta8) + cos(theta4) * cos(theta5) * sin(theta8) - sin(theta4) * sin(theta6 + theta7) * cos(theta8);
            r12_ = -cos(theta4) * sin(theta5) * cos(theta6 + theta7) * sin(theta8) + cos(theta4) * cos(theta5) * cos(theta8) + sin(theta4) * sin(theta6 + theta7) * sin(theta8);
            r13_ = -cos(theta4) * sin(theta5) * sin(theta6 + theta7) - sin(theta4) * cos(theta6 + theta7);
            r21_ =  sin(theta4) * sin(theta5) * cos(theta6 + theta7) * cos(theta8) + sin(theta4) * cos(theta5) * sin(theta8) + cos(theta4) * sin(theta6 + theta7) * cos(theta8);
            r22_ = -sin(theta4) * sin(theta5) * cos(theta6 + theta7) * sin(theta8) + sin(theta4) * cos(theta5) * cos(theta8) - cos(theta4) * sin(theta6 + theta7) * sin(theta8);
            r23_ = -sin(theta4) * sin(theta5) * sin(theta6 + theta7) + cos(theta4) * cos(theta6 + theta7);
            r31_ =  cos(theta5) * cos(theta6 + theta7) * cos(theta8) - sin(theta5) * sin(theta8);
            r32_ = -cos(theta5) * cos(theta6 + theta7) * sin(theta8) - sin(theta5) * cos(theta8);
            r33_ =  -cos(theta5) * sin(theta6 + theta7);
            px_  = -d3 - cos(theta4) * sin(theta5) * sin(theta6) * L6 - sin(theta4) * cos(theta6) * L6 - sin(theta4) * L5;
            py_  =  d2 - sin(theta4) * sin(theta5) * sin(theta6) * L6 + cos(theta4) * cos(theta6) * L6 + cos(theta4) * L5;
            pz_  =  d1 - cos(theta5) * sin(theta6) * L6 + L4;
            px_ = L7 * r13 + px_;
            py_ = L7 * r23 + py_;
            pz_ = L7 * r33 + pz_;
            T06_ = [r11_, r12_, r13_, px_;
                    r21_, r22_, r23_, py_;
                    r31_, r32_, r33_, pz_;
                       0,    0,    0,   1;]

            unequal_count=0;
            for l = 1 : 3
                for m = 1 : 4
                    if (abs(T06_(l, m) - T(l, m)) > 1e-10)
                        unequal_count = unequal_count + 1;
                    end
                end
            end
            if unequal_count == 0
                solution_count = solution_count + 1;
                solution;
            else
                d3 = px - px_
                d2 = py - py_
                d1 = pz - pz_
            end

            d1 = 0; d2 = 0; d3 = 0;
        end
    end
end
solution_count

%% 8轴逆运动学
d1 = 0; d2 = 0; d3 = 0;
% theta4
k = (L6^2 - L5^2 - (px+d3)^2 - (py-d2)^2 - (pz-d1-L4)^2) / (2 * L5);
for i = 1 : 2
    rho = sqrt((px+d3)^2 + (py-d2)^2);
    phi = atan2(py-d2, px+d3);
    sin_theta4_s_phi = k / rho;
    if (abs(1 - abs(sin_theta4_s_phi)) < 1e-10)
        cos_theta4_s_phi = 0;
    else
        if i == 1
            cos_theta4_s_phi = sqrt(1 - sin_theta4_s_phi^2);
        else
            cos_theta4_s_phi = -sqrt(1 - sin_theta4_s_phi^2);
        end
    end
    theta4_s_phi = atan2(cos_theta4_s_phi, sin_theta4_s_phi);
    theta4 = theta4_s_phi + phi;
    sin_theta4 = sin(theta4);
    cos_theta4 = cos(theta4);

    % theta6
    cos_theta6 = (-k-L5) / L6;
    for j = 1 : 2
        if (abs(1 - abs(cos_theta6)) < 1e-10)
            sin_theta6 = 0;
        else
            if j == 1
                sin_theta6 = sqrt(1 - cos_theta6^2);
            else
                sin_theta6 = -sqrt(1 - cos_theta6^2);
            end
        end
        theta6 = atan2(sin_theta6, cos_theta6);

        % theta5
        a = cos_theta4 * (px+d3) + sin_theta4 * (py-d2);
        b = pz - L4 - d1;
        rho = sqrt(a^2 + b^2);
        phi = atan2(b, a);
        for k = 1 : 2
            if (k == 1)
                theta5_a_phi = pi/2;
            else
                theta5_a_phi = -pi/2;
            end
            theta5 = theta5_a_phi - phi;
            sin_theta5 = sin(theta5);
            cos_theta5 = cos(theta5);

            % theta7
            sin_theta6_a_theta7 = -cos_theta4*sin_theta5*r13 - sin_theta4 * sin_theta5*r23 - cos_theta5*r33;
            cos_theta6_a_theta7 = -sin_theta4*r13 + cos_theta4*r23;
            theta7 = atan2(sin_theta6_a_theta7, cos_theta6_a_theta7) - theta6;

            % theta8
            sin_theta8 = cos_theta4*cos_theta5*r11 + sin_theta4*cos_theta5*r21 - sin_theta5*r31;
            cos_theta8 = cos_theta4*cos_theta5*r12 + sin_theta4*cos_theta5*r22 - sin_theta5*r32;

            solution = [d1 d2 d3 theta4 theta5 theta6 theta7 theta8]

            r11_ =  cos(theta4) * sin(theta5) * cos(theta6 + theta7) * cos(theta8) + cos(theta4) * cos(theta5) * sin(theta8) - sin(theta4) * sin(theta6 + theta7) * cos(theta8);
            r12_ = -cos(theta4) * sin(theta5) * cos(theta6 + theta7) * sin(theta8) + cos(theta4) * cos(theta5) * cos(theta8) + sin(theta4) * sin(theta6 + theta7) * sin(theta8);
            r13_ = -cos(theta4) * sin(theta5) * sin(theta6 + theta7) - sin(theta4) * cos(theta6 + theta7);
            r21_ =  sin(theta4) * sin(theta5) * cos(theta6 + theta7) * cos(theta8) + sin(theta4) * cos(theta5) * sin(theta8) + cos(theta4) * sin(theta6 + theta7) * cos(theta8);
            r22_ = -sin(theta4) * sin(theta5) * cos(theta6 + theta7) * sin(theta8) + sin(theta4) * cos(theta5) * cos(theta8) - cos(theta4) * sin(theta6 + theta7) * sin(theta8);
            r23_ = -sin(theta4) * sin(theta5) * sin(theta6 + theta7) + cos(theta4) * cos(theta6 + theta7);
            r31_ =  cos(theta5) * cos(theta6 + theta7) * cos(theta8) - sin(theta5) * sin(theta8);
            r32_ = -cos(theta5) * cos(theta6 + theta7) * sin(theta8) - sin(theta5) * cos(theta8);
            r33_ =  -cos(theta5) * sin(theta6 + theta7);
            px_  = -d3 - cos(theta4) * sin(theta5) * sin(theta6) * L6 - sin(theta4) * cos(theta6) * L6 - sin(theta4) * L5;
            py_  =  d2 - sin(theta4) * sin(theta5) * sin(theta6) * L6 + cos(theta4) * cos(theta6) * L6 + cos(theta4) * L5;
            pz_  =  d1 - cos(theta5) * sin(theta6) * L6 + L4;
            px_ = L7 * r13 + px_;
            py_ = L7 * r23 + py_;
            pz_ = L7 * r33 + pz_;
            T06_ = [r11_, r12_, r13_, px_;
                    r21_, r22_, r23_, py_;
                    r31_, r32_, r33_, pz_;
                       0,    0,    0,   1;]

            unequal_count=0;
            for l = 1 : 3
                for m = 1 : 4
                    if (abs(T06_(l, m) - T(l, m)) > 1e-10)
                        unequal_count = unequal_count + 1;
                    end
                end
            end
            if unequal_count == 0
                solution_count = solution_count + 1;
                solution;
            end
        end
    end
end
solution_count