% Thông số hệ thống
a = 0.255; % Chiều dài robot (m)
b = 0.15; % Chiều rộng robot (m)
r = 0.03; % Bán kính bánh xe (m)
L = sqrt((a/2)^2 + (b/2)^2); % Khoảng cách từ tâm robot đến tâm bánh xe (m)

% Ma trận Jacobian nghịch đảo
J_inv = (1/r) * [1, 1, L;
                 1, -1, -L;
                 1, 1, -L;
                 1, -1, L];

% Thời gian mô phỏng
t1 = 0:0.1:10; % Giai đoạn 1: Đi thẳng 1m
t2 = 10.1:0.1:20; % Giai đoạn 2: Rẽ trái và đi tiếp 1m
t3 = 20.1:0.1:40; % Giai đoạn 3: Quay 1 vòng tròn
t = [t1, t2, t3]; % Tổng thời gian

% Khởi tạo quỹ đạo
X0 = zeros(1, length(t));
Y0 = zeros(1, length(t));
theta = zeros(1, length(t));

% Giai đoạn 1: Đi thẳng 1m
v_x1 = 0.1; % Vận tốc theo trục X (m/s)
v_y1 = 0; % Vận tốc theo trục Y
omega1 = 0; % Vận tốc góc
for i = 1:length(t1)
    X0(i) = v_x1 * t1(i);
    Y0(i) = v_y1 * t1(i);
    theta(i) = omega1 * t1(i);
end

% Giai đoạn 2: Rẽ trái và đi tiếp 1m
v_x2 = 0; % Vận tốc theo trục X
v_y2 = 0.1; % Vận tốc theo trục Y (m/s)
omega2 = 0; % Vận tốc góc
for i = length(t1)+1:length(t1)+length(t2)
    X0(i) = X0(length(t1)) + v_x2 * (t(i) - t(length(t1)));
    Y0(i) = Y0(length(t1)) + v_y2 * (t(i) - t(length(t1)));
    theta(i) = theta(length(t1)) + omega2 * (t(i) - t(length(t1)));
end

% Giai đoạn 3: Quay 1 vòng tròn
R = 0.5; % Bán kính vòng tròn (m)
omega3 = 0.5; % Vận tốc góc (rad/s)
for i = length(t1)+length(t2)+1:length(t)
    X0(i) = X0(length(t1)+length(t2)) + R * cos(omega3 * (t(i) - t(length(t1)+length(t2))));
    Y0(i) = Y0(length(t1)+length(t2)) + R * sin(omega3 * (t(i) - t(length(t1)+length(t2))));
    theta(i) = theta(length(t1)+length(t2)) + omega3 * (t(i) - t(length(t1)+length(t2)));
end

% Tính vận tốc góc của từng bánh xe
omega_wheels = zeros(4, length(t)); % Ma trận lưu vận tốc góc của 4 bánh xe
for i = 1:length(t)
    if i <= length(t1)
        v_robot = [v_x1; v_y1; omega1]; % Giai đoạn 1
    elseif i <= length(t1) + length(t2)
        v_robot = [v_x2; v_y2; omega2]; % Giai đoạn 2
    else
        v_robot = [-R * omega3 * sin(omega3 * (t(i) - t(length(t1)+length(t2))));
                    R * omega3 * cos(omega3 * (t(i) - t(length(t1)+length(t2))));
                    omega3]; % Giai đoạn 3
    end
    omega_wheels(:, i) = J_inv * v_robot; % Vận tốc góc của các bánh xe
end

% Hiển thị kết quả
figure;
plot(X0, Y0, 'b-', 'LineWidth', 2); % Vẽ quỹ đạo
xlabel('X (m)');
ylabel('Y (m)');
title('Quỹ đạo của robot: Đi thẳng, rẽ trái, quay vòng tròn');
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