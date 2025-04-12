clc; clear; close all;

% Thông số robot
r = 0.03;
Lx = 0.075; Ly = 0.125;
R = Lx + Ly;

% Ma trận động học ngược
J_inverse = (1/r) * [... 
    1, -1, -R;
    1,  1,  R;
    1,  1, -R;
    1, -1,  R ];

% Thời gian mô phỏng
T = 31.5;
dt = 0.05;
N = round(T / dt);

% Chuyển động tròn
radius = 1;
v = 0.2;
omega = v / radius;

% Trạng thái ban đầu
x = radius; y = 0; theta = pi/2;
trajectory = zeros(N, 3);
omega_history = zeros(N, 4);

% Tạo figure mô phỏng robot
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Mô phỏng robot Mecanum đi theo đường tròn R = 1m');
xlim([-1.5 1.5]); ylim([-1.5 1.5]);

% Vẽ đường tròn tham chiếu

h_trajectory = plot(x, y, 'b-', 'LineWidth', 1.5);
h_robot = quiver(x, y, cos(theta)*0.2, sin(theta)*0.2, 'r', 'LineWidth', 2);

% Vòng lặp mô phỏng
for k = 1:N
    vx = -v * sin(theta);
    vy =  v * cos(theta);
    wz = omega;

    R_theta = [cos(theta), sin(theta); -sin(theta), cos(theta)];
    v_local = [R_theta * [vx; vy]; wz];

    omega_wheel = J_inverse * v_local;
    omega_history(k, :) = omega_wheel';

    % Ma trận động học thuận
    J_forward = (r/4) * [... 
        1,  1,  1,  1;
       -1,  1,  1, -1;
       -1/R, 1/R, -1/R, 1/R ];

    v_actual = J_forward * omega_wheel;
    v_global = [cos(theta), -sin(theta); sin(theta), cos(theta)] * v_actual(1:2);

    x = x + v_global(1) * dt;
    y = y + v_global(2) * dt;
    theta = theta + v_actual(3) * dt;

    trajectory(k,:) = [x, y, theta];

    set(h_trajectory, 'XData', trajectory(1:k,1), 'YData', trajectory(1:k,2));
    set(h_robot, 'XData', x, 'YData', y, ...
        'UData', cos(theta)*0.2, 'VData', sin(theta)*0.2);
    pause(0.01);
end

% === Vẽ vận tốc góc bánh xe SAU vòng lặp ===
figure;
plot((0:N-1)*dt, omega_history);
xlabel('Thời gian (s)');
ylabel('Tốc độ góc bánh xe (rad/s)');
legend('\omega_1', '\omega_2', '\omega_3', '\omega_4');
title('Vận tốc góc bánh khi đi hình tròn');
grid on;
