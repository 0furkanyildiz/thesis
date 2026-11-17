clear; clc

%% load data

load testData_chirp_input
% load testData_sine_wave_input.mat
% load testData_step_input.mat
% load testData_square_input.mat

%% sim parameters

Ts = 0.01;       % Sampling time [s]
T_end = 60;      % Simulation duration [s]
rng(6);  % Reproducible random noise

%% CT matrices
A_CT = [0       1;
        -k/m   -b/m];

B_CT = [0;
        1/m];

C_CT = [1 0];    % Output: position
D_CT = 0;

%% Convert CT to DT (Zero-Order Hold)
sys_CT = ss(A_CT, B_CT, C_CT, D_CT);
sys_DT = c2d(sys_CT, Ts, 'zoh');

A_DT = sys_DT.A;
B_DT = sys_DT.B;
C_DT = sys_DT.C;
D_DT = sys_DT.D;

%% simulation

t = 0:Ts:T_end;
N = length(t);

%% MIT rule parameters

k_est = 3;
k_min  = 0.01;   % keep the model stable
k_dot = 0;

gamma = 2;

%% storage allocation 

x_model      = zeros(2,N);    % states for the model inside the MIT rule.
x_model(:,1) = [0; 0];

k_est_list = zeros(1,N);
k_est_list(1) = k_est;

k_dot_list = zeros(1,N);
k_dot_list(1) = k_dot;

e_list     = zeros(1,N);

%% initial

phi_dot = 0;
phi = 0;

%% Loop
 
for i = 1:N-1

    %% Adaptation error

    % Use measured position
    x_real = y(i);

    % Reference model position
    x_model_position = x_model(1,i);

    % Tracking error
    e = x_real - x_model_position;

    %% Parameter adaptation

    % Adaptation law
    k_dot = gamma * e * phi;

    % Sensitivity model acceleration
    phi_dot_dot = (-b*phi_dot ...
                   -k_est*phi ...
                   -x_model(1,i))/m;

    %% Reference model dynamics

    A_est = [0          1;
            -k_est/m   -b/m];

    B_est = [0;
             1/m];

    x_model_dot = A_est*x_model(:,i) + B_est*u(i);

    %% Forward Euler integration

    % Parameter update
    k_est = max(k_min, k_est + k_dot*Ts);

    % Sensitivity model update
    phi_dot = phi_dot + phi_dot_dot*Ts;
    phi = phi + phi_dot*Ts;

    % Reference model update
    x_model(:,i+1) = x_model(:,i) + x_model_dot*Ts;

    %% Store results

    k_est_list(i+1) = k_est;
    k_dot_list(i+1) = k_dot;
    e_list(i) = e;

end

figure(1);
plot(t, k_est_list, 'LineWidth', 1.5); 
hold on; 
yline(k, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Stiffness');
title('Stiffness Estimation');
legend('Estimated Stiffness', 'Real Stiffness');

figure(2);
plot(t, e_list, 'LineWidth', 1.5); 
hold on; 
xlabel('Time [s]');
ylabel('Error');
title('Error');
