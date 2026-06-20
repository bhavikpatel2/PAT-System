

Plant_Model;   % gives Ad, Bd, Cd

Ts = 0.001;
Tsim = 0.5;
N = Tsim/Ts;
time = (0:N-1)*Ts;

% ---- LQR weights ----
Q = [1e7 0;
     0   10];

R = 0.1;

% ---- Compute gain ----
K = dlqr(Ad, Bd, Q, R); % LQR function from MATLAB

disp('LQR Gain K = ');
disp(K);

% ---- Simulation ----
x = [5e-3; 0];   % initial error

u_max = 10;

theta_hist = zeros(1,N);
u_hist = zeros(1,N);

for k = 1:N
    
    % LQR control
    u = -K*x;
    
    % Saturation
    u = max(min(u, u_max), -u_max);
    
    % System update
    x = Ad*x + Bd*u;
    
    theta_hist(k) = x(1);
    u_hist(k) = u;
end

% ---- Plot ----
% figure;
% plot(time, theta_hist*1e6, 'LineWidth',2);
% xlabel('Time (s)');
% ylabel('Error (µrad)');
% title('LQR Response');
% grid on;
% 
% figure;
% plot(time, u_hist, 'LineWidth',2);
% xlabel('Time (s)');
% ylabel('Control');
% title('Control Effort');
% grid on;