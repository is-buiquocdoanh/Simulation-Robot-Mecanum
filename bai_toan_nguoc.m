% Mô phỏng bài toán động học ngược cho AGV kiểu bánh xe Mecanum với quỹ đạo đường tròn
clear all; clc;

% Thiết lập tham số
r = 0.05; % Bán kính bánh xe (m)
l = 0.2; % Khoảng cách từ tâm robot đến bánh xe (m)
R = 1; % Bán kính quỹ đạo (m)
omega_c = 0.2; % Tốc độ góc quỹ đạo (rad/s)

% Thời gian mô phỏng
t_start = 0;
t_end = 10 * pi / omega_c; % Thời gian cho 1 vòng tròn
dt = 0.01; % Bước thời gian
t = t_start:dt:t_end;
n_steps = length(t);

% Tính vận tốc trong hệ cố định dựa trên quỹ đạo đường tròn
x_dot = -R * omega_c * sin(omega_c * t); % Vận tốc theo x
y_dot = R * omega_c * cos(omega_c * t); % Vận tốc theo y
theta_dot = omega_c * ones(n_steps, 1); % Vận tốc góc (giữ nguyên hướng)

% Ma trận J (phương trình 1.1, phụ thuộc vào theta ban đầu)
theta_initial = 0; % Góc ban đầu
J = [cos(theta_initial) -sin(theta_initial) 0; ...
     sin(theta_initial)  cos(theta_initial) 0; ...
     0                  0                  1];

% Ma trận H (phương trình 1.8, kích thước 4x3)
H = r * [1  1  -l; ...
         1 -1   l; ...
         1  1   l; ...
         1 -1  -l];

% Ma trận nghịch đảo H^-1 (phương trình 1.10, kích thước 3x4)
H_inv = (1/r) * [1  1  1   1; ...
                 1 -1  1  -1; ...
                -1/l 1/l 1/l -1/l];

% Khởi tạo mảng lưu trữ kết quả
omega1 = zeros(n_steps, 1); % Tốc độ quay bánh xe 1
omega2 = zeros(n_steps, 1); % Tốc độ quay bánh xe 2
omega3 = zeros(n_steps, 1); % Tốc độ quay bánh xe 3
omega4 = zeros(n_steps, 1); % Tốc độ quay bánh xe 4

% Mô phỏng
for i = 1:n_steps
    % Bước 1: Tính vận tốc trong hệ robot [v_x; v_y; omega] = J^T * [x_dot; y_dot; theta_dot] (phương trình 1.1 ngược)
    J_transpose = J'; % Ma trận chuyển vị của J
    fixed_vel = [x_dot(i); y_dot(i); theta_dot(i)];
    robot_vel = J_transpose * fixed_vel; % [v_x; v_y; omega]
    v_x = robot_vel(1);
    v_y = robot_vel(2);
    omega = robot_vel(3);
    
    % Bước 2: Tính vận tốc bánh xe V = H * [v_x; v_y; omega] (phương trình 1.8)
    V = H * [v_x; v_y; omega];
    v1 = V(1);
    v2 = V(2);
    v3 = V(3);
    v4 = V(4);
    
    % Bước 3: Tính tốc độ quay Omega = V / r (phương trình 1.14, đơn giản hóa)
    omega1(i) = v1 / r;
    omega2(i) = v2 / r;
    omega3(i) = v3 / r;
    omega4(i) = v4 / r;
end

% Vẽ kết quả
figure;
subplot(2, 1, 1);
plot(t, omega1, 'b-', 'LineWidth', 2); hold on;
plot(t, omega2, 'r-', 'LineWidth', 2);
plot(t, omega3, 'g-', 'LineWidth', 2);
plot(t, omega4, 'm-', 'LineWidth', 2);
xlabel('Thời gian (s)');
ylabel('Tốc độ quay (rad/s)');
legend('\omega_1', '\omega_2', '\omega_3', '\omega_4');
title('Tốc độ quay của 4 bánh xe');
grid on;

subplot(2, 1, 2);
plot(R * cos(omega_c * t), R * sin(omega_c * t), 'k-', 'LineWidth', 2);
xlabel('x (m)');
ylabel('y (m)');
title('Quỹ đạo đường tròn của robot');
grid on;
axis equal;