%% Stage 3: PID Heading Controller with Anti-Windup
% Replaces the P-only steering of Stage 2 with a full PID controller.
% Integral anti-windup clamps accumulation to [-i_max, +i_max].
% Fixed sliding-window path indexing prevents backward jumps on the loop.
clc; clear; close all;

L       = 0.3;    % wheel baseline (m)
dt      = 0.05;   % time step (s)
T       = 25;     % total time (s)
N       = T / dt;
v_base  = 0.8;    % constant forward speed (m/s)
Ld      = 0.5;    % lookahead distance (m)

% PID gains
Kp    = 3.0;
Ki    = 0.3;
Kd    = 0.08;
i_max = 1.0;    % integral anti-windup limit (rad)

n_path = 800;
t_path = linspace(0, 2*pi, n_path);
path_x = 2.0 * sin(t_path);
path_y = 1.0 * sin(2 * t_path);
path   = [path_x; path_y];

pose     = [0; 0; pi/2];
e_prev   = 0;
e_int    = 0;
path_idx = 1;

hist_pose  = zeros(3, N);
hist_error = zeros(1, N);
hist_terms = zeros(3, N);  % [P; I; D] contributions

for k = 1:N
    hist_pose(:, k) = pose;
    x = pose(1); y = pose(2); theta = pose(3);

    % Sliding-window closest-point search (forward only)
    search_window = 60;
    best_dist = inf;
    best_idx  = path_idx;
    for j = 0:search_window
        idx_j = mod(path_idx + j - 1, n_path) + 1;
        d = sqrt((path(1,idx_j)-x)^2 + (path(2,idx_j)-y)^2);
        if d < best_dist
            best_dist = d;
            best_idx  = idx_j;
        end
    end
    path_idx = best_idx;

    % Lookahead point
    look_idx = path_idx;
    for j = 0:n_path-1
        idx_j = mod(path_idx + j - 1, n_path) + 1;
        if sqrt((path(1,idx_j)-x)^2 + (path(2,idx_j)-y)^2) >= Ld
            look_idx = idx_j;
            break;
        end
    end

    dx        = path(1, look_idx) - x;
    dy        = path(2, look_idx) - y;
    theta_ref = atan2(dy, dx);
    e_theta   = atan2(sin(theta_ref - theta), cos(theta_ref - theta));

    % PID with anti-windup
    e_int   = e_int + e_theta * dt;
    e_int   = max(-i_max, min(i_max, e_int));
    e_dot   = (e_theta - e_prev) / dt;

    term_P  = Kp * e_theta;
    term_I  = Ki * e_int;
    term_D  = Kd * e_dot;
    omega   = term_P + term_I + term_D;

    hist_error(k)   = e_theta;
    hist_terms(:,k) = [term_P; term_I; term_D];
    e_prev          = e_theta;

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
title('Stage 3 — PID Controlled Path Following');
legend('Reference Path', 'Robot Trajectory', 'Start', Location='best');

figure;
plot(t_vec, rad2deg(hist_error), 'b-', LineWidth=1.5);
yline(0, 'k--'); yline(5,'g:',LineWidth=1); yline(-5,'g:',LineWidth=1);
xlabel('Time (s)'); ylabel('Heading Error (deg)');
title('Stage 3 — Heading Error with PID');
legend('PID Error','Zero','±5° band', Location='best');
grid on;

figure;
plot(t_vec, hist_terms(1,:), 'r-', LineWidth=1.5); hold on;
plot(t_vec, hist_terms(2,:), 'g-', LineWidth=1.5);
plot(t_vec, hist_terms(3,:), 'b-', LineWidth=1.5);
xlabel('Time (s)'); ylabel('Control Contribution (rad/s)');
title('Stage 3 — PID Term Breakdown');
legend('Proportional','Integral','Derivative', Location='best');
ylim([-3 3]); grid on;
