clc; clear; close all;

Controller_PID; 

theta_pid = theta_hist;
u_pid = u_hist;
time_pid = time;

clear theta_hist u_hist time


controller_LQR;   

theta_lqr = theta_hist;
u_lqr = u_hist;
time_lqr = time;

time = time_pid;

%% ---- Tracking Comparison ----
figure;
plot(time, theta_pid*1e6, 'b', 'LineWidth',2); hold on;
plot(time, theta_lqr*1e6, 'r--', 'LineWidth',2);
xlabel('Time (s)');
ylabel('Error (µrad)');
title('Tracking: PID vs LQR');
legend('PID','LQR');
grid on;

%% ---- Control Effort Comparison ----
figure;
plot(time, u_pid, 'b', 'LineWidth',2); hold on;
plot(time, u_lqr, 'r--', 'LineWidth',2);
xlabel('Time (s)');
ylabel('Control Input');
title('Control Effort: PID vs LQR');
legend('PID','LQR');
grid on;
