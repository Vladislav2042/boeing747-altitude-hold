% Part C, version 2: improved PID tuning with stability-margin check (altitude hold, Boeing 747)

clear; clc; close all;

%% 1. Longitudinal State-Space Model (from Part A)
A_long = [-0.00643   0.0263    0      -32.2    0;
          -0.0941   -0.624   820       0      0;
          -0.000222 -0.00153 -0.668    0      0;
           0         0        1        0      0;
           0        -1        0       830     0];

B_long = [0; -32.7; -2.08; 0; 0];
C_long = [0 0 0 0 1];
D_long = 0;

[num, den] = ss2tf(A_long, B_long, C_long, D_long);
G_h = tf(num, den);

%% 2. Improved PID Gains
Kp = -0.000300;
Ki = -0.00000500;
Kd = -0.0017;

C_pid = pid(Kp, Ki, Kd);

disp('=== Improved PID Controller ===');
disp(C_pid);

sys_closed = feedback(C_pid * G_h, 1);

poles_cl = pole(sys_closed);
disp('=== Closed-Loop Poles ===');
disp(poles_cl);

if all(real(poles_cl) < 0)
    disp(' The closed-loop system is ASYMPTOTICALLY STABLE');
else
    disp(' WARNING: Unstable poles detected!');
end

S = stepinfo(sys_closed);
disp('=== Step Response Performance ===');
disp(S);

[GM, PM, Wg, Wp] = margin(C_pid * G_h);
fprintf('\n=== Stability Margins ===\n');
fprintf('Gain Margin   = %.4f dB   (at %.2f rad/s)\n', GM, Wg);
fprintf('Phase Margin  = %.2f deg  (at %.2f rad/s)\n', PM, Wp);

figure('Position', [100, 100, 900, 500]);
step(sys_closed, 300);
grid on;
title('Improved PID: Closed-Loop Step Response (Altitude Hold +100 m)');
ylabel('Altitude Change \Delta h (m)');
xlabel('Time (s)');

figure('Position', [1050, 100, 900, 500]);
margin(C_pid * G_h);
grid on;
title('Bode Plot and Stability Margins (Improved PID)');
