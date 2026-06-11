%% Stage 1: Differential Drive Robot — Kinematics
% Open-loop forward simulation of a differential drive robot.
% Both wheel speeds are fixed; the robot traces a circular arc.
clc; clear; close all;

L  = 0.3;   % wheel baseline (m)
dt = 0.05;  % time step (s)
T  = 10;    % total time (s)
N  = T / dt;

pose = [0; 0; 0];  % [x; y; theta]

vL = 0.8;   % left wheel speed (m/s)
vR = 1.0;   % right wheel speed (m/s)

history = zeros(3, N);

for k = 1:N
    history(:, k) = pose;

    v     = (vR + vL) / 2;
    omega = (vR - vL) / L;

    pose(1) = pose(1) + dt * v * cos(pose(3));
    pose(2) = pose(2) + dt * v * sin(pose(3));
    pose(3) = pose(3) + dt * omega;
end

figure;
plot(history(1,:), history(2,:), 'b-', LineWidth=2);
hold on;
plot(history(1,1), history(2,1), 'go', MarkerSize=10, MarkerFaceColor='g');
plot(history(1,end), history(2,end), 'rs', MarkerSize=10, MarkerFaceColor='r');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Stage 1 — Open-Loop Differential Drive (Circular Arc)');
legend('Trajectory', 'Start', 'End', Location='best');
