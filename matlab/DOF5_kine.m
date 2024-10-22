% 计算5轴机械臂(无法使用)
%% 5轴正解
theta1 = 0; theta2 = 0; theta3 = 0; theta4 = 0; theta5 = 0;
L1 = 58; L2 = 119; L3 = 227; L4 = 51.44;

r11 =  cos(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + cos(theta1) * cos(theta2) * sin(theta5) - sin(theta1) * sin(theta3 + theta4) * cos(theta5);
r21 =  sin(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + sin(theta1) * cos(theta2) * sin(theta5) + cos(theta1) * sin(theta3 + theta4) * cos(theta5);
r31 =  cos(theta2) * cos(theta3 + theta4) * cos(theta5) - sin(theta2) * sin(theta5);
r12 = -cos(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + cos(theta1) * cos(theta2) * cos(theta5) + sin(theta1) * sin(theta3 + theta4) * sin(theta5);
r22 = -sin(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + sin(theta1) * cos(theta2) * cos(theta5) - cos(theta1) * sin(theta3 + theta4) * sin(theta5);
r32 = -cos(theta2) * cos(theta3 + theta4) * sin(theta5) - sin(theta2) * cos(theta5);
r13 = -cos(theta1) * sin(theta2) * sin(theta3 + theta4) - sin(theta1) * cos(theta3 + theta4);
r23 = -sin(theta1) * sin(theta2) * sin(theta3 + theta4) + cos(theta1) * cos(theta3 + theta4);
r33 = -cos(theta2) * sin(theta3 + theta4);
px  = -cos(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) - sin(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) - sin(theta1) * L2;
py  = -sin(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + cos(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) + cos(theta1) * L2;
pz  = -cos(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + L1;
% py = 200;
% pz = 0;
T06 = [r11, r12, r13, px;
       r21, r22, r23, py;
       r31, r32, r33, pz;
         0,   0,   0,  1;]
% px = -L4 * r13 + px;
% py = -L4 * r23 + py;
% pz = -L4 * r33 + pz;
% T05 = [r11, r12, r13, px;
%        r21, r22, r23, py;
%        r31, r32, r33, pz;
%          0,   0,   0,  1;]
%% 5轴逆解 L4  = 0 1.0
o = 0;
% theta1
theta1_k = (L3^2 - L2^2 - px^2 - py^2 - (pz - L1)^2) / (2 * L2);
theta1_rho = sqrt(px^2 + py^2);
theta1_phi = atan2(py, px);
sin_theta1_s_phi = theta1_k/theta1_rho; % sin(θ1-φ)
if (abs(1 - abs(sin_theta1_s_phi)) < 1e-10)
    cos_theta1_s_phi = 0;
else
    cos_theta1_s_phi = sqrt(1 - sin_theta1_s_phi^2);
end
theta1_s_phi = [0 0];
theta1_s_phi(1) = atan2(sin_theta1_s_phi, cos_theta1_s_phi);
theta1_s_phi(2) = atan2(sin_theta1_s_phi, -cos_theta1_s_phi);
theta1 = [0 0];
theta1(1) = theta1_s_phi(1) + theta1_phi;
theta1(2) = theta1_s_phi(2) + theta1_phi;
if (theta1(1)>pi)
    theta1(1) = theta1(1) - 2*pi;
elseif (theta1(1)<-pi)
    theta1(1) = theta1(1) + 2*pi;
end
if (theta1(2)>pi)
    theta1(2) = theta1(2) - 2*pi;
elseif (theta1(2)<-pi)
    theta1(2) = theta1(2) + 2*pi;
end
if (abs(theta1(1) - theta1(2)) < 1e-10)
    theta1_num = 1;
else
    theta1_num = 2;
end

% theta3
cos_theta3 = (-theta1_k - L2) / L3;
if (abs(1 - cos_theta3) < 1e-10)
    sin_theta3 = 0;
else
    sin_theta3 = sqrt(1 - cos_theta3^2);
end
theta3 = [0 0];
theta3(1) = atan2(sin_theta3, cos_theta3);
theta3(2) = atan2(-sin_theta3, cos_theta3);
if (abs(theta3(1) - theta3(2)) < 1e-10)
    theta3_num = 1;
else
    theta3_num = 2;
end

% theta4
for i = 1 : theta1_num
    cos_theta3_a_theta4 = cos(theta1(i)) * r23 - sin(theta1(i)) * r13;
    if (abs(1 - cos_theta3_a_theta4) < 1e-10)
        sin_theta3_a_theta4 = 0;
    else
        sin_theta3_a_theta4 = sqrt(1 - cos_theta3_a_theta4^2);
    end

    for j = 1 : 2
        if j == 1 
            theta3_a_theta4 = atan2(sin_theta3_a_theta4, cos_theta3_a_theta4);
        else
            theta3_a_theta4 = atan2(-sin_theta3_a_theta4, cos_theta3_a_theta4);
        end
        for k = 1 : theta3_num
            theta4 = theta3_a_theta4 - theta3(k);

            sin_theta2 = (cos(theta1(i)) * r13 + sin(theta1(i)) * r23) / (-sin_theta3_a_theta4);
            if (abs(1 - abs(sin_theta2)) < 1e-10)
                cos_theta2 = 0;
            else
                cos_theta2 = sqrt(1 - sin_theta2^2);
            end
            for l = 1 : 2
                if l == 1
                    theta2 = atan2(sin_theta2, cos_theta2);
                else
                    theta2 = atan2(sin_theta2, -cos_theta2);
                end

                sin_theta5 = (cos(theta1(i)) * r22 - sin(theta1(i)) * r12) / sin_theta3_a_theta4;
                if (abs(1 - sin_theta5^2) < 1e-10)
                    cos_theta5 = 0;
                else
                    cos_theta5 = sqrt(1 - sin_theta5^2);
                end
                for m = 1 : 2
                    if m == 1
                        theta5 = atan2(sin_theta5, cos_theta5);
                    else
                        theta5 = atan2(sin_theta5, -cos_theta5);
                    end
                    theta_ = [theta1(i) theta2 theta3(k) theta4 theta5]
                end
            end
        end
    end
end
o
%% 5轴逆解 L4  = 0 2.0
o = 0;
% theta1
theta1_k = (L3^2 - L2^2 - px^2 - py^2 - (pz - L1)^2) / (2 * L2);
theta1_rho = sqrt(px^2 + py^2);
theta1_phi = atan2(py, px);
sin_theta1_s_phi = theta1_k / theta1_rho; % sin(θ1-φ)
for i = 1 : 2
    if (abs(1 - abs(sin_theta1_s_phi)) < 1e-10)
        cos_theta1_s_phi = 0;
    else
        if i == 1
            cos_theta1_s_phi = sqrt(1 - sin_theta1_s_phi^2);
        else
            cos_theta1_s_phi = -sqrt(1 - sin_theta1_s_phi^2);
        end
    end
    theta1_s_phi = atan2(sin_theta1_s_phi, cos_theta1_s_phi);
    theta1 = theta1_s_phi + theta1_phi;
    if (theta1 > pi)
        theta1 = theta1 - 2*pi;
    elseif (theta1 < -pi)
        theta1 = theta1 + 2*pi;
    end

    % theta3
    cos_theta3 = (px^2 + py^2 + (pz - L1)^2 - L2^2 - L3^2) / (2 * L2 * L3);
    for j = 1 : 2
        if (abs(1 - abs(cos_theta3)) < 1e-10)
            sin_theta3 = 0;
        else
            if j == 1
                sin_theta3 = sqrt(1 - cos_theta3^2);
            else
                sin_theta3 = -sqrt(1 - cos_theta3^2);
            end
        end
        theta3 = atan2(sin_theta3, cos_theta3);
        if (theta3 > pi)
            theta3 = theta3 - 2*pi;
        elseif (theta3 < -pi)
            theta3 = theta3 + 2*pi;
        end

        % theta4
        cos_theta3_a_theta4 = -sin_theta1 * r13 + cos_theta1 * r23;
        for k = 1 : 2
            if (abs(1 - abs(cos_theta3_a_theta4)) < 1e-10)
                sin_theta3 = 0;
            else
                if k == 1
                    sin_theta3_a_theta4 = sqrt(1 - cos_theta3_a_theta4^2);
                else
                    sin_theta3_a_theta4 = -sqrt(1 - cos_theta3_a_theta4^2);
                end
            end
            theta3_a_theta4 = atan2(sin_theta3_a_theta4, cos_theta3_a_theta4);
            theta4 = theta3_a_theta4 - theta3;
            if (theta4 > pi)
                theta4 = theta4 - 2*pi;
            elseif (theta4 < -pi)
                theta4 = theta4 + 2*pi;
            end

            % theta2
            % sin_theta2 = (cos_theta1 * px + sin_theta1 * py) / (-sin_theta3 * L3);
            % cos_theta2 = (pz - L1) / (-sin_theta3 * L3);
            sin_theta2 = (cos_theta1 * r13 + sin_theta1 * r23) / (-sin(theta3_a_theta4));
            cos_theta2 = r33 / (-sin_theta3_a_theta4);
            theta2 = atan2(sin_theta2,cos_theta2);
            if (theta2 > pi)
                theta2 = theta2 - 2*pi;
            elseif (theta2 < -pi)
                theta2 = theta2 + 2*pi;
            end
            % sin_theta2 = sin(theta2);
            % cos_theta2 = cos(theta2);
    
            % theta5
            % cos_theta5 = (-sin_theta1 * r11 + cos_theta1 * r21) / (sin_theta3_a_theta4);
            % sin_theta5 = (-sin_theta1 * r12 + cos_theta1 * r22) / (-sin_theta3_a_theta4);
            sin_theta5 = r11 * cos_theta1 * cos_theta2 + r21 * sin_theta1 * cos_theta2 - r31 * sin_theta2;
            cos_theta5 = r12 * cos_theta1 * cos_theta2 + r22 * sin_theta1 * cos_theta2 - r32 * sin_theta2;
            theta5 = atan2(sin_theta5, cos_theta5);
            if (theta5 > pi)
                theta5 = theta5 - 2*pi;
            elseif (theta5 < -pi)
                theta5 = theta5 + 2*pi;
            end

            theta_ = [theta1, theta2, theta3, theta4, theta5]

            r11_ =  cos(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + cos(theta1) * cos(theta2) * sin(theta5) - sin(theta1) * sin(theta3 + theta4) * cos(theta5);
            r21_ =  sin(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + sin(theta1) * cos(theta2) * sin(theta5) + cos(theta1) * sin(theta3 + theta4) * cos(theta5);
            r31_ =  cos(theta2) * cos(theta3 + theta4) * cos(theta5) - sin(theta2) * sin(theta5);
            r12_ = -cos(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + cos(theta1) * cos(theta2) * cos(theta5) + sin(theta1) * sin(theta3 + theta4) * sin(theta5);
            r22_ = -sin(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + sin(theta1) * cos(theta2) * cos(theta5) - cos(theta1) * sin(theta3 + theta4) * sin(theta5);
            r32_ = -cos(theta2) * cos(theta3 + theta4) * sin(theta5) - sin(theta2) * cos(theta5);
            r13_ = -cos(theta1) * sin(theta2) * sin(theta3 + theta4) - sin(theta1) * cos(theta3 + theta4);
            r23_ = -sin(theta1) * sin(theta2) * sin(theta3 + theta4) + cos(theta1) * cos(theta3 + theta4);
            r33_ = -cos(theta2) * sin(theta3 + theta4);
            px_  = -cos(theta1) * sin(theta2) * sin(theta3) * L3 - sin(theta1) * cos(theta3) * L3 - sin(theta1) * L2;
            py_  = -sin(theta1) * sin(theta2) * sin(theta3) * L3 + cos(theta1) * cos(theta3) * L3 + cos(theta1) * L2;
            pz_  = -cos(theta2) * sin(theta3) * L3 + L1;
            T05_ = [r11_, r12_, r13_, px_;
                    r21_, r22_, r23_, py_;
                    r31_, r32_, r33_, pz_;
                       0,    0,    0,   1;];

            n=0;
            for l = 1 : 3
                for m = 1 : 4
                    if (abs(T05_(l, m) - T05(l, m)) > 1e-10)
                        n = n + 1;
                    end
                end
            end
            if n == 0
                o = o + 1;
                theta_
            end
        end
    end
end
o
%% 5轴逆解 L4 != 0 1.0
% tehta3
o = 0;
A = px - r13 * L4;  B = py - r23 * L4;  C = pz - L1 - r33 * L4;
cos_theta3 = (A^2 + B^2 + C^2 - L2^2 - L3^2) / (2 * L2 * L3);
for i = 1 : 2
    if (abs(1 - cos_theta3^2) < 1e-10)
        sin_theta3 = 0;
    else
        if i == 1
            sin_theta3 = sqrt(1 - cos_theta3^2);
        else
            sin_theta3 = -sqrt(1 - cos_theta3^2);
        end
    end
    theta3 = atan2(sin_theta3, cos_theta3);

    % tehta2
    if (abs(sin_theta3) < 1e-10)
        cos_theta2 = cos(theta2);
    else
        cos_theta2 = -C/(sin_theta3 * L3);
    end
    for j = 1 : 2
        if (abs(1 - cos_theta2^2) < 1e-10)
            sin_theta2 = 0;
        else
            if j == 1
                sin_theta2 = sqrt(1 - cos_theta2^2);
            else
                sin_theta2 = -sqrt(1 - cos_theta2^2);
            end
        end
        theta2 = atan2(sin_theta2, cos_theta2);


        % % tehta1
        % sin_theta1 = (B * sin_theta2 * sin_theta3 * L3 - A * cos_theta3 * L3 - A * L2) / (A^2 + B^2);
        % cos_theta1 = (A * sin_theta2 * sin_theta3 * L3 + B * cos_theta3 * L3 + B * L2) / (A^2 + B^2);
        % theta1 = atan2(sin_theta1, cos_theta1);
        % 
        % % tehta4
        % sin_theta3_a_theta4 = -r33 / cos_theta2;
        % cos_theta3_a_theta4 = cos_theta1 * r23 - sin_theta1 * r13;
        % theta3_a_theta4 = atan2(sin_theta3_a_theta4, cos_theta3_a_theta4);
        % theta4 = theta3_a_theta4 - theta3;
        % 
        % % tehta5
        % sin_theta5 = (cos_theta2 * cos_theta3_a_theta4 * r23 + sin_theta2 * r13) / (-r13^2 - r23^2);
        % cos_theta5 = (cos_theta2 * cos_theta3_a_theta4 * r13 - sin_theta2 * r23) / (r13^2 + r23^2);
        % theta5 = atan2(sin_theta5, cos_theta5);

        % % tehta1
        % sin_theta1 = (B * sin_theta2 * sin_theta3 * L3 - A * cos_theta3 * L3 - A * L2) / (A^2 + B^2);
        % cos_theta1 = (A * sin_theta2 * sin_theta3 * L3 + B * cos_theta3 * L3 + B * L2) / (A^2 + B^2);
        % theta1 = atan2(sin_theta1, cos_theta1);
        % 
        % % theta4
        % sin_theta4 = r13 * (-cos_theta1 * sin_theta2 * cos_theta3 + sin_theta1 * sin_theta3) + r23 * (-sin_theta1 * sin_theta2 * cos_theta3 - cos_theta1 * sin_theta3) - r33 * cos_theta2 * cos_theta3;
        % cos_theta4 = r13 * (-cos_theta1 * sin_theta2 * sin_theta3 - sin_theta1 * cos_theta3) + r23 * (-sin_theta1 * sin_theta2 * sin_theta3 + cos_theta1 * cos_theta3) - r33 * cos_theta2 * sin_theta3;
        % 
        % % theta5
        % sin_theta5 = r11 * cos_theta1 * cos_theta2 + r21 * sin_theta1 * cos_theta2 - r31 * sin_theta2;
        % cos_theta5 = r12 * cos_theta1 * cos_theta2 + r22 * sin_theta1 * cos_theta2 - r32 * sin_theta2;
        % 
        % theta_ = [theta1 theta2 theta3 theta4 theta5]
        % 
        % r11_ =  cos(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + cos(theta1) * cos(theta2) * sin(theta5) - sin(theta1) * sin(theta3 + theta4) * cos(theta5);
        % r21_ =  sin(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + sin(theta1) * cos(theta2) * sin(theta5) + cos(theta1) * sin(theta3 + theta4) * cos(theta5);
        % r31_ =  cos(theta2) * cos(theta3 + theta4) * cos(theta5) - sin(theta2) * sin(theta5);
        % r12_ = -cos(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + cos(theta1) * cos(theta2) * cos(theta5) + sin(theta1) * sin(theta3 + theta4) * sin(theta5);
        % r22_ = -sin(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + sin(theta1) * cos(theta2) * cos(theta5) - cos(theta1) * sin(theta3 + theta4) * sin(theta5);
        % r32_ = -cos(theta2) * cos(theta3 + theta4) * sin(theta5) - sin(theta2) * cos(theta5);
        % r13_ = -cos(theta1) * sin(theta2) * sin(theta3 + theta4) - sin(theta1) * cos(theta3 + theta4);
        % r23_ = -sin(theta1) * sin(theta2) * sin(theta3 + theta4) + cos(theta1) * cos(theta3 + theta4);
        % r33_ = -cos(theta2) * sin(theta3 + theta4);
        % px_  = -cos(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) - sin(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) - sin(theta1) * L2;
        % py_  = -sin(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + cos(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) + cos(theta1) * L2;
        % pz_  = -cos(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + L1;
        % T06_ = [r11_, r12_, r13_, px_;
        %         r21_, r22_, r23_, py_;
        %         r31_, r32_, r33_, pz_;
        %            0,    0,    0,   1;];
        % 
        % n=0;
        % for l = 1 : 3
        %     for m = 1 : 4
        %         if (abs(T06_(l, m) - T06(l, m)) > 1e-10)
        %             n = n + 1;
        %         end
        %     end
        % end
        % if n == 0
        %     o = o + 1;
        %     theta_;
        % end

        % theta1
        theta1_rho = sqrt(A^2 + B^2);   theta1_phi = atan2(A, B);
        sin_theta1_a_phi = (-sin_theta2 * sin_theta3 * L3) / theta1_rho;
        for k = 1 : 2
            if (abs(1 - sin_theta1_a_phi^2) < 1e-10)
            cos_theta1_a_phi = 0;
            else
                if k == 1
                    cos_theta1_a_phi = sqrt(1 - sin_theta1_a_phi^2);
                else
                    cos_theta1_a_phi = -sqrt(1 - sin_theta1_a_phi^2);
                end
            end
            theta1_s_phi = atan2(sin_theta1_a_phi, cos_theta1_a_phi);
            theta1 = theta1_s_phi - theta1_phi;

            % theta4
            sin_theta4 = r13 * (-cos(theta1) * sin_theta2 * cos_theta3 + sin(theta1) * sin_theta3) + r23 * (-sin(theta1) * sin_theta2 * cos_theta3 - cos(theta1) * sin_theta3) - r33 * cos_theta2 * cos_theta3;
            cos_theta4 = r13 * (-cos(theta1) * sin_theta2 * sin_theta3 - sin(theta1) * cos_theta3) + r23 * (-sin(theta1) * sin_theta2 * sin_theta3 + cos(theta1) * cos_theta3) - r33 * cos_theta2 * sin_theta3;
            theta4 = atan2(sin_theta4, cos_theta4);

            % theta5
            sin_theta5 = r11 * cos(theta1) * cos_theta2 + r21 * sin(theta1) * cos_theta2 - r31 * sin_theta2;
            cos_theta5 = r12 * cos(theta1) * cos_theta2 + r22 * sin(theta1) * cos_theta2 - r32 * sin_theta2;
            theta5 = atan2(sin_theta5, cos_theta5);

            theta_ = [theta1 theta2 theta3 theta4 theta5]

            r11_ =  cos(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + cos(theta1) * cos(theta2) * sin(theta5) - sin(theta1) * sin(theta3 + theta4) * cos(theta5);
            r21_ =  sin(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + sin(theta1) * cos(theta2) * sin(theta5) + cos(theta1) * sin(theta3 + theta4) * cos(theta5);
            r31_ =  cos(theta2) * cos(theta3 + theta4) * cos(theta5) - sin(theta2) * sin(theta5);
            r12_ = -cos(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + cos(theta1) * cos(theta2) * cos(theta5) + sin(theta1) * sin(theta3 + theta4) * sin(theta5);
            r22_ = -sin(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + sin(theta1) * cos(theta2) * cos(theta5) - cos(theta1) * sin(theta3 + theta4) * sin(theta5);
            r32_ = -cos(theta2) * cos(theta3 + theta4) * sin(theta5) - sin(theta2) * cos(theta5);
            r13_ = -cos(theta1) * sin(theta2) * sin(theta3 + theta4) - sin(theta1) * cos(theta3 + theta4);
            r23_ = -sin(theta1) * sin(theta2) * sin(theta3 + theta4) + cos(theta1) * cos(theta3 + theta4);
            r33_ = -cos(theta2) * sin(theta3 + theta4);
            px_  = -cos(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) - sin(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) - sin(theta1) * L2;
            py_  = -sin(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + cos(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) + cos(theta1) * L2;
            pz_  = -cos(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + L1;
            T06_ = [r11_, r12_, r13_, px_;
                   r21_, r22_, r23_, py_;
                   r31_, r32_, r33_, pz_;
                      0,    0,    0,   1;]

            n=0;
            for l = 1 : 3
                for m = 1 : 4
                    if (abs(T06_(l, m) - T06(l, m)) > 1e-10)
                        n = n + 1;
                    end
                end
            end
            if n == 0
                o = o + 1;
                theta_;
            end
        end

        % % tehta1
        % sin_theta1 = (B * sin_theta2 * sin_theta3 * L3 - A * cos_theta3 * L3 - A * L2) / (A^2 + B^2);
        % cos_theta1 = (A * sin_theta2 * sin_theta3 * L3 + B * cos_theta3 * L3 + B * L2) / (A^2 + B^2);
        % theta1 = atan2(sin_theta1, cos_theta1);
        % 
        % % theta4
        % sin_theta3_a_theta4 = r33 / (-cos_theta2);
        % for p = 1 : 2
        %     if (abs(1 - sin_theta3_a_theta4^2) < 1e-10)
        %         cos_theta3_a_theta4 = 0;
        %     else
        %         if p == 1
        %             cos_theta3_a_theta4 = sqrt(1 - sin_theta3_a_theta4^2);
        %         else
        %             cos_theta3_a_theta4 = -sqrt(1 - sin_theta3_a_theta4^2);
        %         end
        %     end
        %     theta3_a_theta4 = atan2(sin_theta3_a_theta4, cos_theta3_a_theta4);
        %     theta4 = theta3_a_theta4 - theta3;
        % 
        %     % theta5
        %     sin_theta5 = r11 * cos_theta1 * cos_theta2 + r21 * sin_theta1 * cos_theta2 - r31 * sin_theta2;
        %     cos_theta5 = r12 * cos_theta1 * cos_theta2 + r22 * sin_theta1 * cos_theta2 - r32 * sin_theta2;
        % 
        %     theta_ = [theta1 theta2 theta3 theta4 theta5];
        % 
        %     r11_ =  cos(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + cos(theta1) * cos(theta2) * sin(theta5) - sin(theta1) * sin(theta3 + theta4) * cos(theta5);
        %     r21_ =  sin(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + sin(theta1) * cos(theta2) * sin(theta5) + cos(theta1) * sin(theta3 + theta4) * cos(theta5);
        %     r31_ =  cos(theta2) * cos(theta3 + theta4) * cos(theta5) - sin(theta2) * sin(theta5);
        %     r12_ = -cos(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + cos(theta1) * cos(theta2) * cos(theta5) + sin(theta1) * sin(theta3 + theta4) * sin(theta5);
        %     r22_ = -sin(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + sin(theta1) * cos(theta2) * cos(theta5) - cos(theta1) * sin(theta3 + theta4) * sin(theta5);
        %     r32_ = -cos(theta2) * cos(theta3 + theta4) * sin(theta5) - sin(theta2) * cos(theta5);
        %     r13_ = -cos(theta1) * sin(theta2) * sin(theta3 + theta4) - sin(theta1) * cos(theta3 + theta4);
        %     r23_ = -sin(theta1) * sin(theta2) * sin(theta3 + theta4) + cos(theta1) * cos(theta3 + theta4);
        %     r33_ = -cos(theta2) * sin(theta3 + theta4);
        %     px_  = -cos(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) - sin(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) - sin(theta1) * L2;
        %     py_  = -sin(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + cos(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) + cos(theta1) * L2;
        %     pz_  = -cos(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + L1;
        %     T06_ = [r11_, r12_, r13_, px_;
        %            r21_, r22_, r23_, py_;
        %            r31_, r32_, r33_, pz_;
        %               0,    0,    0,   1;];
        % 
        %     n=0;
        %     for l = 1 : 3
        %         for m = 1 : 4
        %             if (abs(T06_(l, m) - T06(l, m)) > 1e-10)
        %                 n = n + 1;
        %             end
        %         end
        %     end
        %     if n == 0
        %         o = o + 1;
        %         theta_
        %     end
        % end
    end
end
o

%% 5轴逆解 L4 != 0 2.0 可用
% tehta3
o = 0;
A = px - r13 * L4;  B = py - r23 * L4;  C = pz - L1 - r33 * L4;
cos_theta3 = (A^2 + B^2 + C^2 - L2^2 - L3^2) / (2 * L2 * L3);
for i = 1 : 2
    if (abs(1 - cos_theta3^2) < 1e-10)
        sin_theta3 = 0;
    else
        if i == 1
            sin_theta3 = sqrt(1 - cos_theta3^2);
        else
            sin_theta3 = -sqrt(1 - cos_theta3^2);
        end
    end
    theta3 = atan2(sin_theta3, cos_theta3);

    % theta1
    rho = sqrt(A^2 + B^2);
    phi = atan2(A, B);
    cos_theta1_a_phi = (cos_theta3 * L3 + L2) / rho;
    for j = 1 : 2
        if (abs(1 - abs(cos_theta1_a_phi)) < 1e-10)
            sin_theta1_a_phi = 0;
        else
            if j == 1
                sin_theta1_a_phi = sqrt(1 - cos_theta1_a_phi^2);
            else
                sin_theta1_a_phi = -sqrt(1 - cos_theta1_a_phi^2);
            end
        end
        theta1_s_phi = atan2(sin_theta1_a_phi, cos_theta1_a_phi);
        theta1 = theta1_s_phi - phi;
        sin_theta1 = sin(theta1);
        cos_theta1 = cos(theta1);

        % theta4
        cos_theta3_a_theta4 =  -sin_theta1 * r13 + cos_theta1 * r23;
        for k = 1 : 2
            if (abs(1 - abs(cos_theta3_a_theta4)) < 1e-10)
                sin_theta3_a_theta4 = 0;
            else
                if j == 1
                    sin_theta3_a_theta4 = sqrt(1 - cos_theta3_a_theta4^2);
                else
                    sin_theta3_a_theta4 = -sqrt(1 - cos_theta3_a_theta4^2);
                end
            end
            theta3_a_theta4 = atan2(sin_theta3_a_theta4, cos_theta3_a_theta4);
            theta4 = theta3_a_theta4 - theta3;

            % theta2
            sin_theta2 = (cos_theta1 * r13 + sin_theta1 * r23) / (-sin_theta3_a_theta4);
            cos_theta2 = r33 / (-sin_theta3_a_theta4);
            theta2 = atan2(sin_theta2, cos_theta2);

            % theta5
            sin_theta5 = r11 * cos(theta1) * cos_theta2 + r21 * sin(theta1) * cos_theta2 - r31 * sin_theta2;
            cos_theta5 = r12 * cos(theta1) * cos_theta2 + r22 * sin(theta1) * cos_theta2 - r32 * sin_theta2;
            theta5 = atan2(sin_theta5, cos_theta5);

            theta_ = [theta1 theta2 theta3 theta4 theta5]

            r11_ =  cos(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + cos(theta1) * cos(theta2) * sin(theta5) - sin(theta1) * sin(theta3 + theta4) * cos(theta5);
            r21_ =  sin(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + sin(theta1) * cos(theta2) * sin(theta5) + cos(theta1) * sin(theta3 + theta4) * cos(theta5);
            r31_ =  cos(theta2) * cos(theta3 + theta4) * cos(theta5) - sin(theta2) * sin(theta5);
            r12_ = -cos(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + cos(theta1) * cos(theta2) * cos(theta5) + sin(theta1) * sin(theta3 + theta4) * sin(theta5);
            r22_ = -sin(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + sin(theta1) * cos(theta2) * cos(theta5) - cos(theta1) * sin(theta3 + theta4) * sin(theta5);
            r32_ = -cos(theta2) * cos(theta3 + theta4) * sin(theta5) - sin(theta2) * cos(theta5);
            r13_ = -cos(theta1) * sin(theta2) * sin(theta3 + theta4) - sin(theta1) * cos(theta3 + theta4);
            r23_ = -sin(theta1) * sin(theta2) * sin(theta3 + theta4) + cos(theta1) * cos(theta3 + theta4);
            r33_ = -cos(theta2) * sin(theta3 + theta4);
            px_  = -cos(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) - sin(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) - sin(theta1) * L2;
            py_  = -sin(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + cos(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) + cos(theta1) * L2;
            pz_  = -cos(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + L1;
            T06_ = [r11_, r12_, r13_, px_;
                   r21_, r22_, r23_, py_;
                   r31_, r32_, r33_, pz_;
                      0,    0,    0,   1;]

            n=0;
            for l = 1 : 3
                for m = 1 : 4
                    if (abs(T06_(l, m) - T06(l, m)) > 1e-10)
                        n = n + 1;
                    end
                end
            end
            if n == 0
                o = o + 1;
                theta_;
            end
        end
    end
end
o
%% 5轴逆解 L4 != 0 1.0
% tehta3
o = 0;
A = px - r13 * L4;  B = py - r23 * L4;  C = pz - L1 - r33 * L4;
cos_theta3 = (A^2 + B^2 + C^2 - L2^2 - L3^2) / (2 * L2 * L3);
for i = 1 : 2
    if (abs(1 - cos_theta3^2) < 1e-10)
        sin_theta3 = 0;
    else
        if i == 1
            sin_theta3 = sqrt(1 - cos_theta3^2);
        else
            sin_theta3 = -sqrt(1 - cos_theta3^2);
        end
    end
    theta3 = atan2(sin_theta3, cos_theta3);

    % tehta2
    if (abs(sin_theta3) > 1e-10)
        cos_theta2 = -C/(sin_theta3 * L3);
    end
    for j = 1 : 2
        if (abs(1 - cos_theta2^2) < 1e-10)
            sin_theta2 = 0;
        else
            if j == 1
                sin_theta2 = sqrt(1 - cos_theta2^2);
            else
                sin_theta2 = -sqrt(1 - cos_theta2^2);
            end
        end
        theta2 = atan2(sin_theta2, cos_theta2);
        if (abs(sin_theta3) < 1e-10)
            theta2 = 0;
            sin_theta2 = 0;
            cos_theta2 = 1;
        end

        % tehta1
        sin_theta1 = (B * sin_theta2 * sin_theta3 * L3 - A * cos_theta3 * L3 - A * L2) / (A^2 + B^2);
        cos_theta1 = (A * sin_theta2 * sin_theta3 * L3 + B * cos_theta3 * L3 + B * L2) / (A^2 + B^2);
        theta1 = atan2(sin_theta1, cos_theta1);

        % theta4
        sin_theta4 = r13 * (-cos(theta1) * sin_theta2 * cos_theta3 + sin(theta1) * sin_theta3) + r23 * (-sin(theta1) * sin_theta2 * cos_theta3 - cos(theta1) * sin_theta3) - r33 * cos_theta2 * cos_theta3;
        cos_theta4 = r13 * (-cos(theta1) * sin_theta2 * sin_theta3 - sin(theta1) * cos_theta3) + r23 * (-sin(theta1) * sin_theta2 * sin_theta3 + cos(theta1) * cos_theta3) - r33 * cos_theta2 * sin_theta3;
        theta4 = atan2(sin_theta4, cos_theta4);

        % theta5
        sin_theta5 = r11 * cos(theta1) * cos_theta2 + r21 * sin(theta1) * cos_theta2 - r31 * sin_theta2;
        cos_theta5 = r12 * cos(theta1) * cos_theta2 + r22 * sin(theta1) * cos_theta2 - r32 * sin_theta2;
        theta5 = atan2(sin_theta5, cos_theta5);

        theta_ = [theta1 theta2 theta3 theta4 theta5]

        r11_ =  cos(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + cos(theta1) * cos(theta2) * sin(theta5) - sin(theta1) * sin(theta3 + theta4) * cos(theta5);
        r21_ =  sin(theta1) * sin(theta2) * cos(theta3 + theta4) * cos(theta5) + sin(theta1) * cos(theta2) * sin(theta5) + cos(theta1) * sin(theta3 + theta4) * cos(theta5);
        r31_ =  cos(theta2) * cos(theta3 + theta4) * cos(theta5) - sin(theta2) * sin(theta5);
        r12_ = -cos(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + cos(theta1) * cos(theta2) * cos(theta5) + sin(theta1) * sin(theta3 + theta4) * sin(theta5);
        r22_ = -sin(theta1) * sin(theta2) * cos(theta3 + theta4) * sin(theta5) + sin(theta1) * cos(theta2) * cos(theta5) - cos(theta1) * sin(theta3 + theta4) * sin(theta5);
        r32_ = -cos(theta2) * cos(theta3 + theta4) * sin(theta5) - sin(theta2) * cos(theta5);
        r13_ = -cos(theta1) * sin(theta2) * sin(theta3 + theta4) - sin(theta1) * cos(theta3 + theta4);
        r23_ = -sin(theta1) * sin(theta2) * sin(theta3 + theta4) + cos(theta1) * cos(theta3 + theta4);
        r33_ = -cos(theta2) * sin(theta3 + theta4);
        px_  = -cos(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) - sin(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) - sin(theta1) * L2;
        py_  = -sin(theta1) * sin(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + cos(theta1) * (cos(theta3 + theta4) * L4 + cos(theta3) * L3) + cos(theta1) * L2;
        pz_  = -cos(theta2) * (sin(theta3 + theta4) * L4 + sin(theta3) * L3) + L1;
        T06_ = [r11_, r12_, r13_, px_;
               r21_, r22_, r23_, py_;
               r31_, r32_, r33_, pz_;
                  0,    0,    0,   1;]

        n=0;
        for l = 1 : 3
            for m = 1 : 4
                if (abs(T06_(l, m) - T06(l, m)) > 1e-10)
                    n = n + 1;
                end
            end
        end
        if n == 0
            o = o + 1;
            theta_;
        end
    end
end
o
