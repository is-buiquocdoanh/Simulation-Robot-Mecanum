clc; clear; close all;

% Thông số robot
r = 0.03;           % Bán kính bánh xe (m)
Lx = 0.075; Ly = 0.125; % Khoảng cách tâm robot đến bánh (m)
R = Lx + Ly;

% Ma trận động học thuận
J_forward = (r/4) * [...
     1,  1,  1,  1;
    -1,  1,  1, -1;
    -1/R, 1/R, -1/R, 1/R ];

% Thời gian mô phỏng
T = 20;             % Tổng thời gian (giây)
dt = 0.1;           % Bước thời gian (giây)
N = T / dt;         % Số bước mô phỏng

% Khởi tạo trạng thái ban đầu
x = 0; y = 0; theta = 0;
trajectory = zeros(N, 3);  % Lưu trạng thái

% Tạo figure
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Mô phỏng chuyển động Robot Mecanum');
xlim([-2 2]); ylim([-2 2]);

% Vẽ quỹ đạo ban đầu
h_trajectory = plot(x, y, 'b-', 'LineWidth', 1.5);
% Vẽ hướng robot bằng mũi tên
h_robot = quiver(x, y, cos(theta)*0.2, sin(theta)*0.2, 'r', 'LineWidth', 2);

% Vòng lặp mô phỏng
for k = 1:N
    t = (k-1) * dt;

    % Tốc độ các bánh (có thể thay đổi để tạo đường cong)
    omega = [0; 5; 5; 0]; % ví dụ biến thiên

    % Tính vận tốc robot
    v_robot = J_forward * omega;

    % Chuyển sang hệ cố định
    v_global = [...
        cos(theta), -sin(theta);
        sin(theta),  cos(theta)
    ] * v_robot(1:2);

    % Cập nhật vị trí và góc
    x = x + v_global(1) * dt;
    y = y + v_global(2) * dt;
    theta = theta + v_robot(3) * dt;

    % Lưu quỹ đạo
    trajectory(k, :) = [x, y, theta];

    % Cập nhật đồ họa
    set(h_trajectory, 'XData', trajectory(1:k,1), 'YData', trajectory(1:k,2));
    set(h_robot, 'XData', x, 'YData', y, ...
                 'UData', cos(theta)*0.2, 'VData', sin(theta)*0.2);

    pause(0.01);
end
