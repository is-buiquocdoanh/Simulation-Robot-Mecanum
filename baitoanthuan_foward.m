% Thông số hệ thống
a = 0.255; % Chiều dài robot (m)
b = 0.15; % Chiều rộng robot (m)
r = 0.03; % Bán kính bánh xe (m)
L = sqrt((a/2)^2 + (b/2)^2); % Khoảng cách từ tâm robot đến tâm bánh xe (m)

% Ma trận Jacobian
J = (r/4) * [1, 1, 1, 1;
              1, -1, 1, -1;
              1/L, 1/L, -1/L, -1/L];

% Thời gian mô phỏng
t = 0:0.1:10; % Thời gian từ 0 đến 10 giây, bước 0.1 giây

% Vận tốc góc của các bánh xe (giả sử)
omega1 = 10 * ones(size(t)); % Bánh xe 1 (rad/s)
omega2 = 8 * ones(size(t));  % Bánh xe 2 (rad/s)
omega3 = 10 * ones(size(t)); % Bánh xe 3 (rad/s)
omega4 = 8 * ones(size(t));  % Bánh xe 4 (rad/s)

% Tính vận tốc của robot
v_robot = zeros(3, length(t)); % Ma trận lưu vận tốc của robot
for i = 1:length(t)
    omega_wheels = [omega1(i); omega2(i); omega3(i); omega4(i)]; % Vận tốc góc các bánh xe
    v_robot(:, i) = J * omega_wheels; % Vận tốc của robot
end

% Tích phân để tính vị trí và hướng
X0 = zeros(1, length(t)); % Vị trí X
Y0 = zeros(1, length(t)); % Vị trí Y
theta = zeros(1, length(t)); % Hướng
for i = 2:length(t)
    dt = t(i) - t(i-1); % Bước thời gian
    X0(i) = X0(i-1) + v_robot(1, i) * dt; % Tích phân v_x
    Y0(i) = Y0(i-1) + v_robot(2, i) * dt; % Tích phân v_y
    theta(i) = theta(i-1) + v_robot(3, i) * dt; % Tích phân omega
end

% Hiển thị kết quả
figure;
plot(X0, Y0, 'b-', 'LineWidth', 2); % Vẽ quỹ đạo
xlabel('X (m)');
ylabel('Y (m)');
title('Quỹ đạo của robot (Bài toán động học thuận)');
grid on;
axis equal;

% Hiển thị hướng của robot
figure;
plot(t, theta, 'r-', 'LineWidth', 2); % Vẽ hướng
xlabel('Thời gian (s)');
ylabel('Hướng (rad)');
title('Hướng của robot theo thời gian');
grid on;