%% Stage 5b: Extended Kalman Filter (EKF) — State Estimation
% Sensor noise (Gaussian, sigma_xy=0.09m, sigma_theta=0.055rad) is added
% to the true pose at each step. An EKF fuses the noisy measurement with
% the kinematic prediction (odometry) to produce a cleaner state estimate.
% The controller drives using the EKF estimate, not the raw sensor.
% Final RMS improvement is printed to the command window.
clc; clear; close all;
rng(42);  % fix seed for reproducibility

% ── Parameters ────────────────────────────────────────────────────
L      = 0.3;    % wheel baseline (m)
dt     = 0.05;   % time step (s)
T      = 25;     % total time (s)
N      = T / dt;
n_path = 800;

v_max     = 1.2;   v_min = 0.3;   kappa_max = 1.2;
Kp = 3.0;  Ki = 0.3;  Kd = 0.08;  i_max = 1.0;

% ── Reference Path ────────────────────────────────────────────────
t_path = linspace(0, 2*pi, n_path);
path_x = 2.0 * sin(t_path);
path_y = 1.0 * sin(2 * t_path);
path   = [path_x; path_y];

dx  = gradient(path_x);   dy  = gradient(path_y);
ddx = gradient(dx);       ddy = gradient(dy);
kappa = abs(dx.*ddy - dy.*ddx) ./ (dx.^2 + dy.^2).^1.5;
kappa = min(kappa, kappa_max);

% ── Sensor Noise Parameters ───────────────────────────────────────
sigma_xy    = 0.09;    % position noise std (m)
sigma_theta = 0.055;   % heading noise std (rad)

% ── EKF Matrices ─────────────────────────────────────────────────
H = eye(3);                                          % direct state observation
Q = diag([0.002, 0.002, 0.0008]);                    % process noise covariance
R = diag([sigma_xy^2, sigma_xy^2, sigma_theta^2]);   % measurement noise covariance

x_est = [0; 0; pi/2];   % initial EKF state estimate
P     = eye(3) * 0.1;   % initial covariance

% ── Initial True Pose ─────────────────────────────────────────────
pose     = [0; 0; pi/2];
e_prev   = 0;  e_int = 0;
path_idx = 1;

% ── Storage ───────────────────────────────────────────────────────
hist_true   = zeros(3, N);
hist_noisy  = zeros(3, N);
hist_kalman = zeros(3, N);
hist_error  = zeros(1, N);
hist_speed  = zeros(1, N);
hist_K_diag = zeros(3, N);

% ── EKF + Control Loop ────────────────────────────────────────────
for k = 1:N
    hist_true(:, k) = pose;

    % 1. Noisy sensor measurement
    noise = [sigma_xy * randn; sigma_xy * randn; sigma_theta * randn];
    z     = pose + noise;
    hist_noisy(:, k) = z;

    % 2. Path following using EKF estimate
    x = x_est(1);  y = x_est(2);  theta = x_est(3);

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

    k_cur  = kappa(path_idx);
    v_base = v_max - (v_max - v_min) * (k_cur / kappa_max);
    v_base = max(v_min, min(v_max, v_base));
    hist_speed(k) = v_base;

    dx_     = path(1, look_idx) - x;
    dy_     = path(2, look_idx) - y;
    e_theta = atan2(sin(atan2(dy_, dx_) - theta), cos(atan2(dy_, dx_) - theta));

    e_int  = max(-i_max, min(i_max, e_int + e_theta * dt));
    e_dot  = (e_theta - e_prev) / dt;
    omega  = Kp*e_theta + Ki*e_int + Kd*e_dot;
    e_prev = e_theta;
    hist_error(k) = e_theta;

    vR = v_base + (omega * L / 2);
    vL = v_base - (omega * L / 2);

    % 3. EKF Predict step (odometry as motion model)
    v_cmd = (vR + vL) / 2;
    w_cmd = (vR - vL) / L;
    th    = x_est(3);

    x_pred = x_est + [dt * v_cmd * cos(th);
                       dt * v_cmd * sin(th);
                       dt * w_cmd];

    F = [1, 0, -dt * v_cmd * sin(th);   % Jacobian of motion model
         0, 1,  dt * v_cmd * cos(th);
         0, 0,  1];

    P_pred = F * P * F' + Q;

    % 4. EKF Update step (fuse with noisy measurement)
    S      = H * P_pred * H' + R;
    K      = P_pred * H' / S;
    innov  = z - H * x_pred;
    innov(3) = atan2(sin(innov(3)), cos(innov(3)));  % wrap heading innovation

    x_est  = x_pred + K * innov;
    P      = (eye(3) - K * H) * P_pred;

    hist_kalman(:, k) = x_est;
    hist_K_diag(:, k) = diag(K);

    % 5. Euler integration on TRUE pose (ground truth)
    v = (vR + vL) / 2;
    pose(1) = pose(1) + dt * v * cos(pose(3));
    pose(2) = pose(2) + dt * v * sin(pose(3));
    pose(3) = pose(3) + dt * (vR - vL) / L;
end

t_vec = (0:N-1) * dt;

% ── Plot 1: Trajectory ────────────────────────────────────────────
figure;
plot(path_x, path_y, 'k--', LineWidth=1.2); hold on;
plot(hist_noisy(1,:),  hist_noisy(2,:),  '.', Color=[0.8 0.2 0.2], MarkerSize=3);
plot(hist_true(1,:),   hist_true(2,:),   'b-', LineWidth=2);
plot(hist_kalman(1,:), hist_kalman(2,:), 'g-', LineWidth=1.5);
plot(hist_true(1,1),   hist_true(2,1),   'go', MarkerSize=10, MarkerFaceColor='g');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Stage 5b — Trajectory: True vs Noisy Sensor vs EKF Estimate');
legend('Reference','Noisy sensor','True pose','EKF estimate','Start', Location='best');

% ── Plot 2: X position over time ──────────────────────────────────
figure;
plot(t_vec, hist_noisy(1,:),  'r.', MarkerSize=4); hold on;
plot(t_vec, hist_true(1,:),   'b-', LineWidth=2);
plot(t_vec, hist_kalman(1,:), 'g-', LineWidth=1.8);
xlabel('Time (s)'); ylabel('X position (m)');
title('Stage 5b — X Position: True vs Noisy vs EKF');
legend('Noisy measurement','True pose','EKF estimate', Location='best');
grid on;

% ── Plot 3: Estimation error ──────────────────────────────────────
err_noisy  = sqrt((hist_noisy(1,:)  - hist_true(1,:)).^2 + ...
                  (hist_noisy(2,:)  - hist_true(2,:)).^2);
err_kalman = sqrt((hist_kalman(1,:) - hist_true(1,:)).^2 + ...
                  (hist_kalman(2,:) - hist_true(2,:)).^2);

figure;
plot(t_vec, err_noisy,  'r-', LineWidth=1.2); hold on;
plot(t_vec, err_kalman, 'g-', LineWidth=1.8);
xlabel('Time (s)'); ylabel('Position error (m)');
title('Stage 5b — Estimation Error: Noisy Sensor vs EKF');
legend('Noisy sensor error','EKF error', Location='best');
grid on;

% ── Plot 4: Kalman gain convergence ──────────────────────────────
figure;
plot(t_vec, hist_K_diag(1,:), 'r-', LineWidth=1.5); hold on;
plot(t_vec, hist_K_diag(2,:), 'g-', LineWidth=1.5);
plot(t_vec, hist_K_diag(3,:), 'b-', LineWidth=1.5);
xlabel('Time (s)'); ylabel('Kalman gain K_{ii}');
title('Stage 5b — EKF Gain Convergence');
legend('K_{11} (x)', 'K_{22} (y)', 'K_{33} (\theta)', Location='best');
grid on;

% ── RMS Summary ───────────────────────────────────────────────────
rms_noisy  = sqrt(mean(err_noisy.^2));
rms_kalman = sqrt(mean(err_kalman.^2));
fprintf('\n── EKF Estimation Performance ──────────────\n');
fprintf('RMS error  |  Noisy sensor : %.4f m\n', rms_noisy);
fprintf('RMS error  |  EKF estimate  : %.4f m\n', rms_kalman);
fprintf('Improvement : %.1f%%\n', (1 - rms_kalman/rms_noisy)*100);
fprintf('────────────────────────────────────────────\n');
