% clc; clear; close all;

% System Parameters
wn = 20;          % rad/s
zeta = 0.7;        
Ts = 1e-3;       % 1 kHz or 1 ms

%% System state-space model
% x1= th; (pointing error)
% x2 =d_th; (angular rate)
% d_x2 = -2*zeta*wn*x2 - wn^2 + u 
% X = [x1;x2]
% [d_x1;dx_2]= AX + Bu state space reprentation
% y = CX+ D
% Continuous-time matrices
A = [0 1;
    -wn^2 -2*zeta*wn];

B = [0;
     1];

C = [1 0]; %Cx
D = 0;  

% Continuous system
sys_c = ss(A,B,C,D); % continuous state-space function of Control system toolbox

% Discretize
sys_d = c2d(sys_c, Ts,'zoh'); % Discrete system conversion toolbox (ZOH used since control input is held constant between sampling instants)

Ac = sys_c.A;
Bc = sys_c.B;
Cc = sys_c.C;
Dc = sys_c.D;

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

% Display
disp('Continuous A='); disp(Ac);
disp('Continuous B='); disp(Bc);
disp('Continuous C='); disp(Cc);
disp('Continuous D='); disp(Dc);

disp('Discrete A='); disp(Ad);
disp('Discrete B='); disp(Bd);
disp('Discrete C='); disp(Cd);
disp('Discrete D='); disp(Dd);

%% Comparison: Continuous vs Discrete

t = 0:Ts:1;

% ---- Initial Condition Response (more relevant) ----
x0 = [5e-3; 0];   % 5 mrad initial error

[y_c, t_c] = initial(sys_c, x0, t);
[y_d, t_d] = initial(sys_d, x0, t);

figure;
plot(t_c, y_c*1e3, 'b', 'LineWidth',2); hold on;
plot(t_d, y_d*1e3, 'r--', 'LineWidth',2);
legend('Continuous','Discrete');
title('Initial Condition Response Comparison (Pointing Error)');
xlabel('Time (s)');
ylabel('Pointing Error (mrad)');
grid on;

% % ---- Step Response ----
% figure;
% step(sys_c, 'b', sys_d, 'r--', t);
% legend('Continuous','Discrete');
% title('Step Response Comparison');
% xlabel('Time (s)');
% ylabel('Output');
% grid on;