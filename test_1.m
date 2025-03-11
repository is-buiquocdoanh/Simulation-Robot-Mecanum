% Thông số hệ thống
R = 1; % Bán kính quỹ đạo tròn (m)
omega_trajectory = 0.5; % Vận tốc góc của quỹ đạo (rad/s)
r = 0.03; % Bán kính bánh xe (m)
L = 0.5; % Khoảng cách từ tâm robot đến tâm bánh xe (m)

% Thời gian mô phỏng
t = 0:0.1:15.7; % Thời gian từ 0 đến 10 giây, bước 0.1 giây

% Quỹ đạo tròn
X0 = R * cos(omega_trajectory * t);
Y0 = R * sin(omega_trajectory * t);

% Vận tốc của robot
dX0 = -R * omega_trajectory * sin(omega_trajectory * t);
dY0 = R * omega_trajectory * cos(omega_trajectory * t);
dTheta = omega_trajectory; % Vận tốc góc của robot (giả sử không đổi)

% Ma trận Jacobian nghịch đảo (4x3)
J_inv = (1/r) * [1, 1, L;
                 1, -1, -L;
                 1, 1, -L;
                 1, -1, L];

% Tính vận tốc góc của từng bánh xe
omega_wheels = zeros(4, length(t)); % Ma trận lưu vận tốc góc của 4 bánh xe
for i = 1:length(t)
    v_robot = [dX0(i); dY0(i); dTheta]; % Vận tốc của robot (3x1)
    omega_wheels(:, i) = J_inv * v_robot; % Vận tốc góc của các bánh xe
end

% Hiển thị kết quả
figure;
plot(X0, Y0, 'b-', 'LineWidth', 2); % Vẽ quỹ đạo tròn
xlabel('X (m)');
ylabel('Y (m)');
title('Quỹ đạo tròn của robot');
grid on;
axis equal;

% Hiển thị vận tốc góc của các bánh xe
figure;
plot(t, omega_wheels(1, :), 'r-', 'LineWidth', 2); hold on;
plot(t, omega_wheels(2, :), 'g-', 'LineWidth', 2);
plot(t, omega_wheels(3, :), 'b-', 'LineWidth', 2);
plot(t, omega_wheels(4, :), 'm-', 'LineWidth', 2);
xlabel('Thời gian (s)');
ylabel('Vận tốc góc (rad/s)');
title('Vận tốc góc của các bánh xe');
legend('Bánh xe 1', 'Bánh xe 2', 'Bánh xe 3', 'Bánh xe 4');
grid on;