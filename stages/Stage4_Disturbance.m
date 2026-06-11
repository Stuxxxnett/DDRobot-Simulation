%% Stage 4: Disturbance Injection — PID vs P-only Recovery
% Two wheel-speed disturbances are injected at t=5s and t=14s by scaling
% one wheel's speed to simulate slipping or surface friction asymmetry.
% PID (full) vs P-only controllers are run in parallel for comparison.
% Metrics plotted: trajectory, heading error, and cross-track error (CTE).
clc; clear; close all;

L       = 0.3;    % wheel baseline (m)
dt      = 0.05;   % time step (s)
T       = 25;     % total time (s)
N       = T / dt;
v_base  = 0.8;    % forward speed (m/s)
Ld      = 0.5;    % lookahead distance (m)
n_path  = 800;

% PID gains
Kp    = 3.0;
Ki    = 0.3;
Kd    = 0.08;
i_max = 1.0;

t_path = linspace(0, 2*pi, n_path);
path_x = 2.0 * sin(t_path);
path_y = 1.0 * sin(2 * t_path);
path   = [path_x; path_y];

% Disturbance table: [t_start, t_end, vL_scale, vR_scale]
disturbances = [
     5.0,  5.8,  0.2,  1.0;   % left wheel slips
    14.0, 14.8,  1.0,  0.2;   % right wheel slips
];

%% ── PID Run ──────────────────────────────────────────────────────
pose     = [0; 0; pi/2];
e_prev   = 0; e_int = 0;
path_idx = 1;
hist_pid      = zeros(3, N);
hist_err_pid  = zeros(1, N);
hist_disturb  = zeros(1, N);

for k = 1:N
    hist_pid(:, k) = pose;
    x = pose(1); y = pose(2); theta = pose(3);
    t = (k-1) * dt;

    best_dist = inf; best_idx = path_idx;
    for j = 0:60
        idx_j = mod(path_idx + j - 1, n_path) + 1;
        d = sqrt((path(1,idx_j)-x)^2 + (path(2,idx_j)-y)^2);
        if d < best_dist; best_dist = d; best_idx = idx_j; end
    end
    path_idx = best_idx;

    look_idx = path_idx;
    for j = 0:n_path-1
        idx_j = mod(path_idx + j - 1, n_path) + 1;
        if sqrt((path(1,idx_j)-x)^2 + (path(2,idx_j)-y)^2) >= Ld
            look_idx = idx_j; break;
        end
    end

    e_theta = atan2(sin(atan2(path(2,look_idx)-y, path(1,look_idx)-x) - theta), ...
                    cos(atan2(path(2,look_idx)-y, path(1,look_idx)-x) - theta));

    e_int = max(-i_max, min(i_max, e_int + e_theta * dt));
    omega = Kp*e_theta + Ki*e_int + Kd*(e_theta - e_prev)/dt;
    e_prev = e_theta;
    hist_err_pid(k) = e_theta;

    vR = v_base + (omega * L / 2);
    vL = v_base - (omega * L / 2);

    d_scale_L = 1.0; d_scale_R = 1.0;
    for d = 1:size(disturbances,1)
        if t >= disturbances(d,1) && t <= disturbances(d,2)
            d_scale_L = disturbances(d,3);
            d_scale_R = disturbances(d,4);
            hist_disturb(k) = 1;
        end
    end
    vL = vL * d_scale_L;
    vR = vR * d_scale_R;

    v = (vR+vL)/2;
    pose(1) = x + dt*v*cos(theta);
    pose(2) = y + dt*v*sin(theta);
    pose(3) = theta + dt*(vR-vL)/L;
end

%% ── P-Only Run ───────────────────────────────────────────────────
pose     = [0; 0; pi/2];
path_idx = 1;
hist_nopid     = zeros(3, N);
hist_err_nopid = zeros(1, N);

for k = 1:N
    hist_nopid(:, k) = pose;
    x = pose(1); y = pose(2); theta = pose(3);
    t = (k-1) * dt;

    best_dist = inf; best_idx = path_idx;
    for j = 0:60
        idx_j = mod(path_idx + j - 1, n_path) + 1;
        d = sqrt((path(1,idx_j)-x)^2 + (path(2,idx_j)-y)^2);
        if d < best_dist; best_dist = d; best_idx = idx_j; end
    end
    path_idx = best_idx;

    look_idx = path_idx;
    for j = 0:n_path-1
        idx_j = mod(path_idx + j - 1, n_path) + 1;
        if sqrt((path(1,idx_j)-x)^2 + (path(2,idx_j)-y)^2) >= Ld
            look_idx = idx_j; break;
        end
    end

    e_theta = atan2(sin(atan2(path(2,look_idx)-y, path(1,look_idx)-x) - theta), ...
                    cos(atan2(path(2,look_idx)-y, path(1,look_idx)-x) - theta));
    hist_err_nopid(k) = e_theta;

    omega = 2.0 * e_theta;  % proportional only, Kp = 2.0
    vR = v_base + (omega * L / 2);
    vL = v_base - (omega * L / 2);

    d_scale_L = 1.0; d_scale_R = 1.0;
    for d = 1:size(disturbances,1)
        if t >= disturbances(d,1) && t <= disturbances(d,2)
            d_scale_L = disturbances(d,3);
            d_scale_R = disturbances(d,4);
        end
    end
    vL = vL * d_scale_L;
    vR = vR * d_scale_R;

    v = (vR+vL)/2;
    pose(1) = x + dt*v*cos(theta);
    pose(2) = y + dt*v*sin(theta);
    pose(3) = theta + dt*(vR-vL)/L;
end

%% ── Plots ────────────────────────────────────────────────────────
t_vec = (0:N-1)*dt;

figure;
plot(path_x, path_y, 'k--', LineWidth=1.5); hold on;
plot(hist_nopid(1,:), hist_nopid(2,:), 'r-', LineWidth=1.8);
plot(hist_pid(1,:),   hist_pid(2,:),   'b-', LineWidth=2);
for d = 1:size(disturbances,1)
    k1 = max(1, round(disturbances(d,1)/dt));
    k2 = min(N, round(disturbances(d,2)/dt));
    plot(hist_pid(1,k1:k2), hist_pid(2,k1:k2), 'm-', LineWidth=4);
end
plot(hist_pid(1,1), hist_pid(2,1), 'go', MarkerSize=10, MarkerFaceColor='g');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Stage 4 — Disturbance: PID vs No-PID');
legend('Reference','No PID (P-only)','With PID','Disturbance active','Start', Location='best');

figure;
for d = 1:size(disturbances,1)
    patch([disturbances(d,1) disturbances(d,2) disturbances(d,2) disturbances(d,1)], ...
          [-200 -200 200 200], [1 0.8 0.8], EdgeColor='none', FaceAlpha=0.4);
    hold on;
end
plot(t_vec, rad2deg(hist_err_nopid), 'r-', LineWidth=1.5);
plot(t_vec, rad2deg(hist_err_pid),   'b-', LineWidth=1.5);
yline(0,'k--'); yline(5,'g:',LineWidth=1); yline(-5,'g:',LineWidth=1);
ylim([-200 200]);
xlabel('Time (s)'); ylabel('Heading Error (deg)');
title('Stage 4 — Heading Error: PID vs No-PID (pink = disturbance active)');
legend('Disturbance window','No PID','With PID','Zero','±5° band', Location='best');
grid on;

figure;
cte_pid   = zeros(1,N);
cte_nopid = zeros(1,N);
for k = 1:N
    dists = sqrt((path(1,:)-hist_pid(1,k)).^2   + (path(2,:)-hist_pid(2,k)).^2);
    cte_pid(k) = min(dists);
    dists = sqrt((path(1,:)-hist_nopid(1,k)).^2 + (path(2,:)-hist_nopid(2,k)).^2);
    cte_nopid(k) = min(dists);
end
for d = 1:size(disturbances,1)
    patch([disturbances(d,1) disturbances(d,2) disturbances(d,2) disturbances(d,1)], ...
          [0 0 2 2], [1 0.8 0.8], EdgeColor='none', FaceAlpha=0.4); hold on;
end
plot(t_vec, cte_nopid, 'r-', LineWidth=1.5); hold on;
plot(t_vec, cte_pid,   'b-', LineWidth=1.5);
xlabel('Time (s)'); ylabel('Cross-track Error (m)');
title('Stage 4 — Cross-Track Error: PID vs No-PID');
legend('Disturbance window','No PID','With PID', Location='best');
grid on;
