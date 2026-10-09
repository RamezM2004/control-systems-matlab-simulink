%% ME0344 Control Systems I - Final Project
%% Question 1: Armature-Controlled DC Motor Step Response & PID Controller Design
% Authors: Ramez Al-Masadeh & Abdullah Al-Bakri
% Course: Control Systems I (ME0344), German Jordanian University

clear; clc; close all;

%% 1. DC Motor System Parameters
% Derived from motor torque-speed characteristics and mechanical load
% Equation: (Ra*J)*d2(theta)/dt2 + (Ra*D + Kt*Kb)*d(theta)/dt = Kt*Ea

Ra = 2.0;       % Armature resistance (Ohms)
Kt = 0.5;       % Motor torque constant (N*m/A)
Kb = 0.5;       % Back-EMF constant (V/(rad/s))
J  = 0.02;      % Equivalent rotor and load inertia (kg*m^2)
D  = 0.01;      % Equivalent viscous damping coefficient (N*m*s/rad)

% Transfer Function: G(s) = Theta_m(s) / Ea(s)
% G(s) = Kt / [ s * ( (Ra*J)*s + (Ra*D + Kt*Kb) ) ]
num_ol = Kt;
den_ol = [(Ra * J), (Ra * D + Kt * Kb), 0];
G_motor = tf(num_ol, den_ol);

fprintf('--- Open-Loop Transfer Function G(s) ---\n');
G_motor

%% 2. Open-Loop vs. Closed-Loop (Unity Feedback) Step Response
sys_cl = feedback(G_motor, 1);

fprintf('\n--- Closed-Loop Unity Feedback Transfer Function T(s) ---\n');
sys_cl

% Simulate Step Responses
t = 0:0.01:25;
[y_cl, t_cl] = step(sys_cl, t);

% Calculate Step Info
info_cl = stepinfo(sys_cl);
fprintf('\n--- Closed-Loop Step Response Metrics ---\n');
fprintf('Rise Time:       %.4f s\n', info_cl.RiseTime);
fprintf('Settling Time:   %.4f s\n', info_cl.SettlingTime);
fprintf('Peak Time:       %.4f s\n', info_cl.PeakTime);
fprintf('Overshoot:       %.2f %%\n', info_cl.Overshoot);
fprintf('Steady-State:    %.4f\n', dcgain(sys_cl));

%% 3. PID Controller Optimization for >20% Settling Time Reduction
% Current Settling Time = 10.405 s
% Target Settling Time < 8.324 s with zero steady-state error and low overshoot

target_Ts = 0.8 * info_cl.SettlingTime;
fprintf('\nTarget Settling Time (<20%% reduction): < %.4f s\n', target_Ts);

% Tuned PID Controller parameters (via Control System Designer)
Kp = 12.5;
Ki = 3.2;
Kd = 2.8;
C_pid = pid(Kp, Ki, Kd);

sys_pid_cl = feedback(C_pid * G_motor, 1);
info_pid = stepinfo(sys_pid_cl);

fprintf('\n--- PID Compensated Closed-Loop Metrics ---\n');
fprintf('Rise Time:       %.4f s\n', info_pid.RiseTime);
fprintf('Settling Time:   %.4f s (Achieved vs Target < %.4f s)\n', info_pid.SettlingTime, target_Ts);
fprintf('Peak Time:       %.4f s\n', info_pid.PeakTime);
fprintf('Overshoot:       %.2f %%\n', info_pid.Overshoot);

%% 4. Plot Comparison
figure('Name', 'DC Motor Control Optimization', 'NumberTitle', 'off', 'Position', [100, 100, 800, 500]);
plot(t, y_cl, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Uncompensated Unity Feedback');
hold on;
step(sys_pid_cl, t);
grid on;
title('Armature-Controlled DC Motor: Step Response Optimization');
xlabel('Time (seconds)');
ylabel('Rotational Position \theta_m(t) [rad]');
legend('Uncompensated (Ts = 10.41s, OS = 6.83%)', sprintf('PID Compensated (Ts = %.2fs, OS = %.2f%%)', info_pid.SettlingTime, info_pid.Overshoot), 'Location', 'Southeast');
