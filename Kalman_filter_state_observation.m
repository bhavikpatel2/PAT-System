clc; clear; close all;

Plant_Model;

Ts = 0.001;
Tsim = 0.5;
N = Tsim/Ts;
time = (0:N-1)*Ts;

% Noise
sigma = 50e-6;
Rk = sigma^2;
Qk = diag([1e-10 1e-6]);

% Initial conditions
x = [5e-3; 0];        % true state
x_hat = [0; 0];       % estimated state
d_prev =0;
P = eye(2);           % covariance

% PID Controller
Kp = 2000; Ki = 350; Kd = 55;

theta_prev = 0;
integral = 0;

% Storage
x_hist = zeros(2,N);
xhat_hist = zeros(2,N);

for k = 1:N
    
    % ---- Measurement ----
    y = Cd*x + sigma*randn;
    
    % ---- Control algorithm ----
    theta = x_hat(1);
    integral = integral + theta*Ts;
   alpha = 0.5;  % filter strength
    derivative = alpha * d_prev + (1-alpha)*(theta - theta_prev)/Ts;
    
    u = -(Kp*theta + Ki*integral + Kd*derivative);
       u = min(max(u,-u_max),u_max); % Saturation
    % ---- True system ----
    x = Ad*x + Bd*u;
    
    % ---- Kalman Prediction ----
    x_pred = Ad*x_hat + Bd*u;
    P_pred = Ad*P*Ad' + Qk;
    
    % ---- Kalman Update ----
    Kk = P_pred*Cd' / (Cd*P_pred*Cd' + Rk);
    
    x_hat = x_pred + Kk*(y - Cd*x_pred);
    P = (eye(2) - Kk*Cd)*P_pred;
    
    % Store
    x_hist(:,k) = x;
    xhat_hist(:,k) = x_hat;
    
    theta_prev = theta;
    d_prev = derivative;
end

% ---- Plot ----
figure;
plot(time, x_hist(1,:)*1e6, 'LineWidth',2); hold on;
plot(time, xhat_hist(1,:)*1e6,'--','LineWidth',2);
legend('True','Estimated');
xlabel('Time (s)');
ylabel('Position (µrad)');
title('Kalman Filter: Position');
grid on;

figure;
plot(time, x_hist(2,:), 'LineWidth',2); hold on;
plot(time, xhat_hist(2,:),'--','LineWidth',2);
legend('True','Estimated');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
title('Kalman Filter: Velocity');
grid on;