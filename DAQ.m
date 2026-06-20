clc; clear; close all;

% Import plant model
Plant_Model;   % gives Ad, Bd, Cd

% Simulation parameters
Ts = 1e-3;
Tsim = 0.5;
N = Tsim / Ts;
time = (0:N-1)*Ts;

% Initial condition (±5 mrad)
x = [5e-3; 0];

% Acquisition parameters
Ka = 2;                 % coarse gain
threshold = 0.5e-3;     % 0.5 mrad capture region

% Smooth saturation parameter
delta = 0.5e-3;

% Actuator limit
u_max = 10;

% Storage
theta_hist = zeros(1,N);
% u_hist = zeros(1,N);

for k = 1:N
    
    theta = x(1);
    
    % ---- Acquisition Control ----
    if abs(theta) > threshold
        % Smooth (instead of sign)
        sat_theta = max(min(theta/delta, 1), -1);
        u = -Ka * sat_theta;
    else
        % Inside capture region → stop (handover to PID later)
        u = 0;
    end
    
    % ---- Saturation ----
    u = max(min(u, u_max), -u_max);
    
    % ---- System update ----
    x = Ad*x + Bd*u;
    
    % Store
    theta_hist(k) = theta;
    % u_hist(k) = u;
end

%% ---- Plot: Pointing Error ----
figure;
plot(time, theta_hist*1e3, 'LineWidth',2);
xlabel('Time (s)');
ylabel('Pointing Error (mrad)');
title('Acquisition Phase (Coarse Control)');
grid on;