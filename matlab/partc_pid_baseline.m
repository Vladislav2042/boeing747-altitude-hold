% Part C, version 1: first PID tuning attempt (altitude hold, Boeing 747)

%% Part C: Altitude Hold Control System Design for Boeing 747 (PID Controller)
clear; clc; close all;

A_long = [-0.00643  0.0263   0     -32.2    0;
          -0.0941  -0.624  820     0      0;
          -0.000222 -0.00153 -0.668  0      0;
           0        0       1       0      0;
           0       -1       0      830     0];

B_long = [0; -32.7; -2.08; 0; 0];
C_long = [0 0 0 0 1];
D_long = 0;

[num, den] = ss2tf(A_long, B_long, C_long, D_long);
G_h = tf(num, den);

disp('Open-loop Transfer Function h(s)/δe(s):');
G_h

%% 2. PID Controller Parameters
Kp = -0.00022;
Ki = -0.0000035;
Kd = -0.0018;

C_pid = pid(Kp, Ki, Kd);

disp('Designed PID Controller:');
C_pid

sys_closed = feedback(C_pid * G_h, 1);

figure('Name','Step Response - Altitude Hold', 'Position',[100 100 850 520]);
step(sys_closed, 350);
grid on;
title('Closed-loop Step Response - Altitude Hold (\Delta h = +100 m)');
ylabel('Altitude Change \Delta h (m)');
xlabel('Time (s)');

figure('Name','Bode Plot with Stability Margins', 'Position',[1000 100 850 520]);
margin(C_pid * G_h);
grid on;
title('Bode Plot and Stability Margins with PID Controller');


poles_closed = pole(sys_closed);
disp('Closed-loop System Poles:');
disp(poles_closed);

if all(real(poles_closed) < 0)
    disp('The closed-loop system is ASYMPTOTICALLY STABLE');
else
    disp(' The system has unstable poles');
end

S = stepinfo(sys_closed);
disp('Performance Metrics:');
disp(S);
