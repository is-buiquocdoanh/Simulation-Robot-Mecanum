% Thông số hệ thống
A = 1; % Biên độ của quỹ đạo hình sin (m)
omega_trajectory = 1; % Tần số góc của quỹ đạo (rad/s)
r = 0.03; % Bán kính bánh xe (m)
L = 0.148; % Khoảng cách từ tâm robot đến tâm bánh xe (m)

% Ma trận Jacobian nghịch đảo
J_inv = (1/r) * [1, 1, L;
                 1, -1, -L;
                 1, 1, -L;
                 1, -1, L];

% Ma trận Jacobian
J = (r/4) * [1, 1, 1, 1;
             1, -1, 1, -1;
             1/L, 1/L, -1/L, -1/L];

% Thời gian mô phỏng
t = 0:0.1:10; % Thời gian từ 0 đến 10 giây, bước 0.1 giây

% Quỹ đạo hình sin
X0 = t;
Y0 = A * sin(omega_trajectory * t);

% Vận tốc của robot
v_x = ones(size(t)); % Vận tốc theo trục X là hằng số
v_y = A * omega_trajectory * cos(omega_trajectory * t); % Vận tốc theo trục Y
omega = zeros(size(t)); % Vận tốc góc của robot (giả sử không đổi)

% Tính vận tốc góc của các bánh xe
omega_wheels = zeros(4, length(t)); % Ma trận lưu vận tốc góc của 4 bánh xe
for i = 1:length(t)
    v_robot = [v_x(i); v_y(i); omega(i)]; % Vận tốc của robot (3x1)
    omega_wheels(:, i) = J_inv * v_robot; % Vận tốc góc của các bánh xe
end

% Sử dụng bài toán động học thuận để kiểm tra
v_robot_check = zeros(3, length(t)); % Ma trận lưu vận tốc kiểm tra
for i = 1:length(t)
    v_robot_check(:, i) = J * omega_wheels(:, i); % Vận tốc của robot tính từ bài toán thuận
end

% Hiển thị kết quả
figure;
plot(t, v_x, 'r-', 'LineWidth', 2); hold on;
plot(t, v_robot_check(1, :), 'b--', 'LineWidth', 2);
xlabel('Thời gian (s)');
ylabel('Vận tốc v_x (m/s)');
title('So sánh v_x ban đầu và v_x tính từ bài toán thuận');
legend('v_x ban đầu', 'v_x tính từ bài toán thuận');
grid on;

figure;
plot(t, v_y, 'r-', 'LineWidth', 2); hold on;
plot(t, v_robot_check(2, :), 'b--', 'LineWidth', 2);
xlabel('Thời gian (s)');
ylabel('Vận tốc v_y (m/s)');
title('So sánh v_y ban đầu và v_y tính từ bài toán thuận');
legend('v_y ban đầu', 'v_y tính từ bài toán thuận');
grid on;