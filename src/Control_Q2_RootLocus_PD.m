%% ME0344 Control Systems I - Final Project
%% Question 2: Root Locus Analysis & PD Compensator Design
% Authors: Ramez Al-Masadeh & Abdullah Al-Bakri
% Course: Control Systems I (ME0344), German Jordanian University

clear; clc; close all;

%% 1. Plant Transfer Function
% G(s) = 6 / [ s * (2s + 2) * (3s + 24) ] = 1 / [ s * (s + 1) * (s + 8) ]
num = 1;
den = conv([1 0], conv([1 1], [1 8])); % s*(s+1)*(s+8) = s^3 + 9s^2 + 8s
G_plant = tf(num, den);

fprintf('--- Plant Transfer Function G(s) ---\n');
G_plant

%% 2. Root Locus of Uncompensated System
figure('Name', 'Uncompensated Root Locus', 'NumberTitle', 'off');
rlocus(G_plant);
grid on;
title('Root Locus of Uncompensated Plant G(s) = 1 / [s(s+1)(s+8)]');

% Why Proportional Control Alone CANNOT achieve Ts < 2.07 sec:
% Settling time requirement: Ts = 4 / (zeta * wn) < 2.07 s => sigma = zeta * wn > 1.932
% For G(s), the real-axis asymptote centroid is:
% sigma_a = (0 - 1 - 8) / 3 = -3.0
% The breakaway point between s=0 and s=-1 is at s ~= -0.46.
% As K increases, the complex branches curve into the right-half plane and cross the imaginary axis.
% Under proportional gain alone, gain margin and damping constraints make it impossible
% to place dominant poles deep enough in the left half-plane (sigma > 1.93) without causing instability or severe oscillation.

%% 3. PD Compensator Design
% Requirements:
% 1) Dominant time constant tau = 1/(zeta*wn) <= 0.5 s => sigma >= 2.0
% 2) Damping ratio zeta >= 0.707 (Overshoot <= 4.3%)

% PD controller adds a zero: Gc(s) = K * (s + z_pd)
% Zero selected at z_pd = 2.5 to pull root locus branches significantly leftward:
z_pd = 2.5;
Gc_unscaled = tf([1 z_pd], 1);
G_open_pd = Gc_unscaled * G_plant;

figure('Name', 'Compensated Root Locus', 'NumberTitle', 'off');
rlocus(G_open_pd);
grid on;
title('Root Locus with PD Zero at s = -2.5: G_c(s)G(s)');

% Select loop gain K to place dominant poles at desired damping and settling:
K_pd = 16.0;
sys_cl_uncomp = feedback(G_plant, 1);
sys_cl_pd     = feedback(K_pd * G_open_pd, 1);

info_uncomp = stepinfo(sys_cl_uncomp);
info_pd     = stepinfo(sys_cl_pd);

fprintf('\n--- Step Response Comparison ---\n');
fprintf('Uncompensated: Ts = %.4f s, Overshoot = %.2f %%\n', info_uncomp.SettlingTime, info_uncomp.Overshoot);
fprintf('PD Compensated: Ts = %.4f s, Overshoot = %.2f %%\n', info_pd.SettlingTime, info_pd.Overshoot);

%% 4. Step Response Comparison Plot
figure('Name', 'Step Response Comparison', 'NumberTitle', 'off');
step(sys_cl_uncomp, 'r--');
hold on;
step(sys_cl_pd, 'b-');
grid on;
title('Step Response: Uncompensated vs. PD Compensated Plant');
legend('Uncompensated (Proportional Feedback)', 'PD Compensated [G_c(s) = 16(s + 2.5)]', 'Location', 'Southeast');
