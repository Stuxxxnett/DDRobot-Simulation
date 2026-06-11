%% Stage 5a: Speed-Adaptive PID (Curvature-Based Speed Control)
% Forward speed is modulated by local path curvature: high on straights,
% low on tight bends. This reduces heading error at curves without
% changing PID gains.
clc; clear; close all;

L      = 0.3;    % wheel baseline (m)
dt     = 0.05;   % time step (s)
T      = 25;     % total time (s)
N      = T / dt;
n_path = 800;

v_max     = 1.2;   % max speed on straights (m/s)
v_min     = 0.3;   % min speed at sharpest curves (m/s)
kappa_max = 1.2;   % curvature normalisation cap (1/m)

% PID gains (same as Stage 3)
Kp = 3.0;  Ki = 0.3;  Kd = 0.08;  i_max = 1.0;

t_path = linspace(0, 2*pi, n_path);
path_x = 2.0 * sin(t_path);
path_y = 1.0 * sin(2 * t_path);
path   = [path_x; path_y];

% Pre-compute curvature at each path point
dx  = gradient(path_x);
dy  = gradient(path_y);
ddx = gradient(dx);
ddy = gradient(dy);
kappa = abs(dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^1.5;
kappa = min(kappa, kappa_max);

pose     = [0; 0; pi/2];
e_prev   = 0;  e_int = 0;
path_idx = 1;

hist_pose  = zeros(3, N);
hist_error = zeros(1, N);
hist_speed = zeros(1, N);
hist_kappa = zeros(1, N);

for k = 1:N
    hist_pose(:, k) = pose;
    x = pose(1);  y = pose(2);  theta = pose(3);

    best_dist = inf;  best_idx = path_idx;
    for j = 0:60
        idx_j = mod(path_idx + j - 1, n_path) + 1;
        d = sqrt((path(1,idx_j)-x)^2 + (path(2,idx_j)-y)^2);
        if d < best_dist;  best_dist = d;  best_idx = idx_j;  end
    end
    path_idx = best_idx;

    look_idx = path_idx;
    for j = 0:n_path-1
        idx_j = mod(path_idx + j - 1, n_path) + 1;
        if sqrt((path(1,idx_j)-x)^2 + (path(2,idx_j)-y)^2) >= 0.5
            look_idx = idx_j;  break;
        end
    end

    % Curvature-based speed scheduling
    k_cur  = kappa(path_idx);
    v_base = v_max - (v_max - v_min) * (k_cur / kappa_max);
    v_base = max(v_min, min(v_max, v_base));

    hist_speed(k) = v_base;
    hist_kappa(k) = k_cur;

    dx_     = path(1, look_idx) - x;
    dy_     = path(2, look_idx) - y;
    e_theta = atan2(sin(atan2(dy_, dx_) - theta), cos(atan2(dy_, dx_) - theta));

    e_int   = max(-i_max, min(i_max, e_int + e_theta * dt));
    e_dot   = (e_theta - e_prev) / dt;
    omega   = Kp*e_theta + Ki*e_int + Kd*e_dot;
    e_prev  = e_theta;
    hist_error(k) = e_theta;

    vR = v_base + (omega * L / 2);
    vL = v_base - (omega * L / 2);

    v       = (vR + vL) / 2;
    pose(1) = x + dt * v * cos(theta);
    pose(2) = y + dt * v * sin(theta);
    pose(3) = theta + dt * (vR - vL) / L;
end

t_vec = (0:N-1) * dt;

figure;
plot(path_x, path_y, 'k--', LineWidth=1.5); hold on;
plot(hist_pose(1,:), hist_pose(2,:), 'b-', LineWidth=2);
plot(hist_pose(1,1), hist_pose(2,1), 'go', MarkerSize=10, MarkerFaceColor='g');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Stage 5a — Speed-Adaptive PID Trajectory');
legend('Reference','Robot','Start', Location='best');

figure;
plot(t_vec, hist_speed, 'r-', LineWidth=1.8); hold on;
yline(v_max, 'k--', 'v_{max}', LineWidth=1, LabelHorizontalAlignment='left');
yline(v_min, 'k:',  'v_{min}', LineWidth=1, LabelHorizontalAlignment='left');
xlabel('Time (s)'); ylabel('v_{base} (m/s)');
title('Stage 5a — Commanded Speed (dips at curves, peaks on straights)');
ylim([0 1.4]); grid on;

figure;
yyaxis left
plot(t_vec, hist_speed, 'r-', LineWidth=1.5);
ylabel('Speed (m/s)'); ylim([0 1.4]);
yyaxis right
plot(t_vec, rad2deg(hist_error), 'b-', LineWidth=1.2);
ylabel('Heading Error (deg)'); ylim([-40 40]);
yline(0, 'k--');
xlabel('Time (s)');
title('Stage 5a — Speed vs Heading Error (speed drops tame the error)');
legend('Speed','Heading error', Location='best');
grid on;
