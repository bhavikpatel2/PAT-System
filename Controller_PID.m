

% Load plant
Plant_Model;

% Simulation parameters
Tsim = 0.5; % total 1 sec
Ts = 1e-3; % sample time 1 ms
N = Tsim / Ts; % number of iteration
d_prev=0;

% PID gains
Kp = 2000;
Ki = 350;
Kd = 55;

% Initialize
x = [5e-3; 0];   % 5 mrad initial error
theta_prev = x(1);
integral = 0;

theta_hist = zeros(1,N);
u_hist = zeros(1,N);
time = (0:N-1)*Ts;
u_max = 10; % torque saturation
for k = 1:N

    theta = x(1);

    % PID terms
    integral = integral + theta * Ts;
    alpha = 0.5;  % derivative filter gain
    derivative = alpha * d_prev + (1-alpha)*(theta - theta_prev)/Ts;

    u = - (Kp*theta + Ki*integral + Kd*derivative);
    u = min(max(u,-u_max),u_max); % Saturation
    % Update system
    x = Ad*x + Bd*u;

    % Store
    theta_hist(k) = theta;
    u_hist(k) = u;

    theta_prev = theta;
    d_prev = derivative;
end

% % Plot
% figure;
% plot(time, theta_hist*1e6, 'LineWidth',2);
% xlabel('Time (s)');
% ylabel('Error (urad)');
% title('PID Tracking Performance');
% grid on;
% 
% figure;
% plot(time, u_hist, 'LineWidth',2);
% xlabel('Time (s)');
% ylabel('Control Effort');
% title('PID Control Effort');
% grid on;