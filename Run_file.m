clc; clear; close all;

Qcar.M = 300; 
Qcar.m = 50;  
Qcar.K1 = 3000;
Qcar.K2 = 30000; 
Qcar.D = 600;

t0 = 0; 
dt = 0.01; 
Tf = 30;
tspan = t0:dt:Tf; 
t_lenght = length(t0:dt:Tf);

%% Response to The Initial Condition

T  = [t0 Tf];
x0 = [0.2;0;0;0]; 
% y_er = 0.1*(sin(5*tspan) + sin(9*tspan)+ sin(13*tspan) + sin(17*tspan));
y_er = 0 *ones(1,t_lenght);
u = 0 *ones(1,t_lenght);

opts = odeset('RelTol',1e-9,'AbsTol',1e-12);
[t, X] = ode45(@(t,x) QuarterCarState(t, x, Qcar, u, y_er, tspan), T, x0, opts);

figure('Position', [50, 50, 1500, 800]);
tiledlayout(1, 2, 'TileSpacing', 'Compact', 'Padding', 'Compact');

% Plot: Displacement
nexttile;
plot(t, X(:,1), 'r', t, X(:,2), 'b--', 'LineWidth', 3);
grid on;
legend('Sprung mass displacement', 'Unsprung mass displacement', ...
       'NumColumns', 2, 'Location', 'best');
set(gca, 'fontsize', 20);
xlabel('Time (s)');
ylabel('Displacement (m)');
xlim([t0, Tf]);

% Plot: Velocity
nexttile;
plot(t, X(:,3), 'r', t, X(:,4), 'b--', 'LineWidth', 3);
grid on;
legend('Sprung mass velocity', 'Unsprung mass velocity', ...
       'NumColumns', 2, 'Location', 'best');
set(gca, 'fontsize', 20);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
xlim([t0, Tf]);

%% System Dynamics

A = [0                 0                           1                0
     0                 0                           0                1
    -Qcar.K1/Qcar.M    Qcar.K1/Qcar.M             -Qcar.D/Qcar.M    Qcar.D/Qcar.M
     Qcar.K1/Qcar.m   -(Qcar.K1+ Qcar.K2)/Qcar.m   Qcar.D/Qcar.m   -Qcar.D/Qcar.m];

B = [0
     0
     1/Qcar.M
    -1/Qcar.m];

C = [1  0   0   0];

D = [0];

[num, den] = ss2tf(A,B,C,D);

% Create transfer function from state-space
G = tf(num,den);
s = tf('s');

%% P Controller

N = 2000;
Kp_scale = 10;

% Run P controller
[Kp_vec, y_final_P, ss_error_perc_P, STime_P, Oshoot_P] = P_controller_sweep(G, N, Kp_scale);

% Plot
figure;
tiledlayout(3, 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');

% Plot: Settling Time
nexttile;
plot(Kp_vec, STime_P, 'LineWidth', 3); 
grid on;
xlabel('Kp'); 
ylabel('Settling Time (s)'); 
title('P Controller Settling Time vs Kp');
set(gca, 'fontsize', 20);

% Plot: Overshoot
nexttile;
plot(Kp_vec, Oshoot_P, 'LineWidth', 3); 
grid on;
xlabel('Kp'); 
ylabel('Overshoot (%)'); 
title('P Controller Overshoot vs Kp');
set(gca, 'fontsize', 20);

% Plot: Steady-State Error
nexttile;
plot(Kp_vec, ss_error_perc_P, 'LineWidth', 3); 
grid on;
xlabel('Kp'); 
ylabel('Steady-state error (%)'); 
title('P Controller Steady-State Error vs Kp');
set(gca, 'fontsize', 20);

%% PI Controller

N1 = 250;
Kp_scale = 10;
Ki_scale = 10;

% Run PI controller
[Kp_PI, Ki_PI, y_final_PI, ss_error_perc_PI, STime_PI, Oshoot_PI] = PI_controller_sweep(G, N1, Kp_scale, Ki_scale);

% Find optimal gains
[Kp_opt, Ki_opt, T_PI_opt, ~, ~] = PI_optimal(G, Kp_PI, Ki_PI, STime_PI, Oshoot_PI); 

eps_val = 1e-3;  % adding a small number to avoid log(0)
figure('Position',[100 100 1400 700]);
tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');

% Plot: Settling Time
nexttile(1, [1 1]);
Z1 = STime_PI;
Z1(isnan(Z1)) = NaN;
imagesc(Ki_PI(1,:), Kp_PI(:,1), log(Z1));
axis xy; 
xlabel('Ki'); ylabel('Kp');
title('Log of PI Controller Settling Time');
colorbar;
set(gca,'fontsize',20);
colormap(gca,parula);
set(gca,'Color',[0.8 0.8 0.8]);

% Plot: Overshoot
nexttile(3, [1 1]);
Z2 = Oshoot_PI + eps_val;
Z2(isnan(Oshoot_PI)) = NaN;
imagesc(Ki_PI(1,:), Kp_PI(:,1), log(Z2));
axis xy; 
xlabel('Ki'); ylabel('Kp');
title('Log of PI Controller Overshoot');
colorbar;
set(gca,'fontsize',20);
colormap(gca,parula);
set(gca,'Color',[0.8 0.8 0.8]);

% Plot: Step Response
nexttile(2, [2 1]);
if ~isempty(T_PI_opt)
    [y_step, t_step] = step(T_PI_opt, tspan);

    plot(t_step, y_step, 'LineWidth', 3); hold on;
    yline(1, '--', 'LineWidth', 3, 'Color', 'r'); 
    grid on;
    xlabel('Time (s)');
    ylabel('Output (x1)');
    title(sprintf('Step Response of PI Controller: Kp = %.2f, Ki = %.2f', Kp_opt, Ki_opt));
    hold off;
    set(gca,'fontsize',20);
    legend('System Response', 'Reference', ...
           'NumColumns', 2, 'Location', 'best');
else
    text(0.5,0.5,'No feasible PI controller found','HorizontalAlignment','center','FontSize',16);
    axis off;
end

%% PID Controller

% Tune Controller
[C_PID, T_PID, ST_opt, OS_opt] = PID_optimal_tuner(G);

% Step response
[y_step, t_step] = step(T_PID, tspan);
info_PID = stepinfo(T_PID);

ST = info_PID.SettlingTime;
OS = info_PID.Overshoot;
y_final = y_step(end);
ss_error = 100*(1 - y_final);   % % error assuming unit step

% Display results
fprintf('PID Controller Metrics:\n');
fprintf('Settling Time = %.2f s\n', ST);
fprintf('Overshoot = %.2f %%\n', OS);
fprintf('Steady-State Error = %.2f %%\n', ss_error);

% Plot: Step Response
plot(t_step, y_step, 'LineWidth', 3); hold on;
yline(1, '--', 'LineWidth', 3, 'Color', 'r'); 
grid on;
xlabel('Time (s)');
ylabel('Output (x1)');
title(sprintf('Step Response of PI Controller: Kp = %.2f, Ki = %.2f, Kd = %.2f', C_PID.Kp, C_PID.Ki, C_PID.Kd));
hold off;
set(gca,'fontsize',20);
legend('System Response', 'Reference', ...
       'NumColumns', 2, 'Location', 'best');

%% Controllability Check

Co = ctrb(A, B);
n = size(A,1);
r = rank(Co);

if r == n
    disp('System is fully controllable');
else
    disp('System is Not fully controllable');
end

%% SVFC Controller

% Intended poles
poles = [-5 -6 -7 -8];

% State feedback gain
K = place(A, B, poles);

% Compute Nbar for reference tracking
sys_cl_noN = ss(A, B, C, D);
Nbar = 1 / (C * inv(-A + B*K) * B);

% Closed-loop system
sys_SVFC = ss(A - B*K, B*Nbar, C, D);

% Initial condition
u = ones(size(tspan)); % step input

% Simulate response
[y_svfc, t_svfc] = lsim(sys_SVFC, u, tspan, x0);

% Step response metrics
info_svfc = stepinfo(y_svfc, t_svfc, 1);
ST_svfc  = info_svfc.SettlingTime;
OS_svfc  = info_svfc.Overshoot;
SSE_svfc = abs(1 - y_svfc(end)) * 100;

% Plot
figure;
plot(t_svfc, y_svfc, 'LineWidth', 3); hold on;
yline(1,'--r','LineWidth',3);
grid on;
xlabel('Time (s)');
ylabel('Output x_1');
title('Step Response – State Vector Feedback Control');
set(gca,'fontsize',20);

%% Check Observability

Obs = obsv(A,C);
rank_Obs = rank(Obs);
if rank_Obs == length(A)
    disp('System is fully observable')
else
    disp('System is not fully observable')
end

%% SFVC with Observer

poles_observer = 2*[-5 -6 -7 -8]; 
L = place(A', C', poles_observer)';  % note the transpose trick
% Initial conditions
xhat0 = [0; 0; 0; 0];          % observer

x_aug0 = [x0; xhat0];          % combined vector

% Define augmented dynamics
function dx_aug = ObserverDynamics(t, x_aug, A, B, C, u, tspan, L)
    x = x_aug(1:4);
    xhat = x_aug(5:8);
    
    ut = interp1(tspan, u, t);  % input at time t
    y = C*x;                    % actual output
    
    dx = A*x + B*ut;
    dxhat = A*xhat + B*ut + L*(y - C*xhat);
    
    dx_aug = [dx; dxhat];
end

% Simulate
[t, X_aug] = ode45(@(t,x) ObserverDynamics(t, x, A, B, C, u, tspan, L), tspan, x_aug0);

% Separate actual and estimated states
x_real  = X_aug(:,1:4);
x_hat   = X_aug(:,5:8);

% --- Tile plot for state estimation ---
figure('Position', [100 100 1200 800]);
tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');

state_labels = {'x_1 (Vehicle disp)','x_2 (Axle disp)','x_3 (Vehicle vel)','x_4 (Axle vel)'};

for k = 1:4
    nexttile;
    plot(tspan, x_real(:,k), 'b', 'LineWidth', 3); hold on;
    plot(tspan, x_hat(:,k), 'r--', 'LineWidth', 3);
    grid on;
    xlabel('Time (s)');
    ylabel(state_labels{k});
    legend('Actual','Estimated','Location','best');
    set(gca,'fontsize',20);
end

sgtitle('State Estimation with Full-Order Observer','FontSize',20);

