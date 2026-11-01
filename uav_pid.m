% =========================================================================
% TÍNH TOÁN PID CHO DRONE QUADCOPTER 450mm - 1.05kg
% =========================================================================
clc; clear; close all;

%% 1. THÔNG SỐ VẬT LÝ (Cập nhật)
m = 1.05;           % Tổng khối lượng (kg) - Đã cập nhật
L = 0.450 / 2;      % Chiều dài cánh tay (m) - Từ tâm tới motor
g = 9.81;           % Gia tốc trọng trường

% Ước lượng Mô-men quán tính cho khung 450mm (1kg)
% Công thức thực nghiệm cho thấy khung 450mm thường có Ixx, Iyy ~ 0.012 - 0.018
Ixx = 0.015;        % Mô-men quán tính trục Roll (kg.m^2)
Iyy = 0.015;        % Mô-men quán tính trục Pitch (kg.m^2)
Izz = 0.025;        % Mô-men quán tính trục Yaw (kg.m^2) - Yaw thường lớn hơn

%% 2. MÔ HÌNH TOÁN HỌC (PLANT MODEL)
s = tf('s');
G_roll = 1 / (Ixx * s^2); % Động lực học góc

T_motor = 0.05;           % Độ trễ motor (50ms)
G_motor = 1 / (T_motor * s + 1);

Plant = G_motor * G_roll; % Hàm truyền tổng hợp

%% 3. TÍNH TOÁN PID BẰNG PIDTUNE
wc = 12; % Tần số cắt (Bandwidth) - Tăng lên 12 rad/s để drone phản hồi nhanh hơn
opts = pidtuneOptions('DesignFocus', 'reference-tracking');
[C_pid, info] = pidtune(Plant, 'PID', wc, opts);

% Trích xuất hệ số liên tục
Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;

%% 4. RỜI RẠC HÓA PID (Ts = 1ms = 1000Hz)
Ts = 0.001;
C_pid_discrete = c2d(C_pid, Ts, 'tustin');

Kp_d = C_pid_discrete.Kp;
Ki_d = C_pid_discrete.Ki;
Kd_d = C_pid_discrete.Kd;

fprintf('--- KẾT QUẢ PID CHO CODE C (uav_test.c) ---\n');
fprintf('Kp = %.4f\n', Kp_d);
fprintf('Ki = %.4f\n', Ki_d);
fprintf('Kd = %.4f\n', Kd_d);

%% 5. VẼ BIỂU ĐỒ KIỂM TRA
Sys_cl = feedback(C_pid * Plant, 1);
figure('Name', 'Đáp ứng góc Roll - 1.05kg');
step(Sys_cl, 1);
title('Đáp ứng bước (Step Response) - Trục Roll (1.05kg)');
grid on;

S = stepinfo(Sys_cl);
fprintf('\n--- CHẤT LƯỢNG HỆ THỐNG ---\n');
fprintf('Độ vọt lố (Overshoot): %.2f%%\n', S.Overshoot);
fprintf('Thời gian xác lập: %.4f giây\n', S.SettlingTime);z