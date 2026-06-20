clc; clear; close all;

Plant_Model;

Ts = 0.001;
Tsim = 1;
N = Tsim/Ts;
time = (0:N-1)*Ts;

%% ---- PID ----
Kp = 2000;
Ki = 350;
Kd = 55;

Tf = 0.002;

%% ---- Kalman ----
sigma = 50e-6;
Rk = sigma^2;
Qk = diag([1e-10 1e-6]);

x = [0; 0];
x_hat = [0; 0];
P = eye(2);

theta_prev = 0;
integral = 0;
d_prev = 0;

%% ---- Disturbance ----
A_dist = 0.5e-3;   % 0.5 mrad
f = 3;             % 3 Hz

u_max = 10;

theta_hist = zeros(1,N);

for k = 1:N
    
    t = time(k);
    
    % ---- disturbance ----
    d = A_dist * sin(2*pi*f*t);
    
    % ---- measurement ----
    y = Cd*x + sigma*randn;
    
    % ---- PID using estimated state ----
    theta = x_hat(1);

    integral = integral + theta * Ts;
    alpha = 0.5;  % derivative filter gain
    derivative = alpha * d_prev + (1-alpha)*(theta - theta_prev)/Ts;

    u = - (Kp*theta + Ki*integral + Kd*derivative);
    
    % saturation
    u = max(min(u, u_max), -u_max);
    
    % ---- plant with disturbance ----
    x = Ad*x + Bd*(u + d);
    
    % ---- Kalman ----
    x_pred = Ad*x_hat + Bd*u;
    P_pred = Ad*P*Ad' + Qk;
    
    Kk = P_pred*Cd'/(Cd*P_pred*Cd' + Rk);
    
    x_hat = x_pred + Kk*(y - Cd*x_pred);
    P = (eye(2) - Kk*Cd)*P_pred;
    
    theta_hist(k) = x(1);
    
    theta_prev = theta;
    d_prev = derivative;
end

%% ---- Plot ----
figure;
plot(time, theta_hist*1e6, 'LineWidth',2);
xlabel('Time (s)');
ylabel('Error (µrad)');
title('Disturbance Rejection (Using PID and Kalman)');
grid on;