%% Stage 2: Reference Path + Pure Pursuit Steering
% A figure-8 reference path is defined parametrically.
% A lookahead-based Pure Pursuit controller steers the robot along it.
% This stage shows the residual heading error that PID will later correct.
clc; clear; close all;

L       = 0.3;    % wheel baseline (m)
dt      = 0.05;   % time step (s)
T       = 20;     % total time (s)
N       = T / dt;
v_base  = 0.8;    % constant forward speed (m/s)
Ld      = 0.6;    % lookahead distance (m)

t_path  = linspace(0, 2*pi, 500);
path_x  = 2.0 * sin(t_path);
path_y  = 1.0 * sin(2 * t_path);
path    = [path_x; path_y];

pose    = [0; 0; pi/2];  % [x; y; theta]

hist_pose   = zeros(3, N);
hist_error  = zeros(1, N);

for k = 1:N
    hist_pose(:, k) = pose;
    x = pose(1); y = pose(2); theta = pose(3);

    dists    = sqrt((path(1,:) - x).^2 + (path(2,:) - y).^2);
    [~, idx] = min(dists);

    look_idx = idx;
    for j = idx:size(path,2)
        if sqrt((path(1,j)-x)^2 + (path(2,j)-y)^2) >= Ld
            look_idx = j;
            break;
        end
    end

    dx        = path(1, look_idx) - x;
    dy        = path(2, look_idx) - y;
    theta_ref = atan2(dy, dx);

    e_theta       = atan2(sin(theta_ref - theta), cos(theta_ref - theta));
    hist_error(k) = e_theta;

    Kp_steer = 2.0;
    omega    = Kp_steer * e_theta;

    vR = v_base + (omega * L / 2);
    vL = v_base - (omega * L / 2);

    v       = (vR + vL) / 2;
    pose(1) = x + dt * v * cos(theta);
    pose(2) = y + dt * v * sin(theta);
    pose(3) = theta + dt * (vR - vL) / L;
end

figure;
plot(path_x, path_y, 'k--', LineWidth=1.5); hold on;
plot(hist_pose(1,:), hist_pose(2,:), 'b-', LineWidth=2);
plot(hist_pose(1,1), hist_pose(2,1), 'go', MarkerSize=10, MarkerFaceColor='g');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Stage 2 — Pure Pursuit Path Following (Figure-8)');
legend('Reference Path', 'Robot Trajectory', 'Start', Location='best');

figure;
t_vec = (0:N-1) * dt;
plot(t_vec, rad2deg(hist_error), 'r-', LineWidth=1.5);
yline(0, 'k--');
xlabel('Time (s)'); ylabel('Heading Error (deg)');
title('Stage 2 — Heading Error (residual that PID will correct)');
grid on;
