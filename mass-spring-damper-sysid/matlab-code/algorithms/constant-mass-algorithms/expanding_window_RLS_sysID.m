clear; clc

%% load data

load testData_chirp_input
% load testData_sine_wave_input.mat
% load testData_step_input.mat
% load testData_square_input.mat

%% simulation length

N = length(t);

%% RLS 

X = [];
Y = [];

%% Discrete-time simulation using for loop
for i = 2:N-1
    
    x1_RLS = x(1, i-1);
    x2_RLS = x(2, i-1);
    u_RLS  = u(i-1);

    x2_next_RLS = x(2, i);

    Y = [Y; x1_RLS, x2_RLS, u_RLS];
    X = [X; x2_next_RLS];

    % window least squares
    theta = pinv(Y) * X;
    
    % Recover physical parameters
    m_est = Ts / theta(3);
    b_est = (1 - theta(2)) * m_est / Ts;
    k_est = -theta(1) * m_est / Ts;

    m_est_list(i) = m_est;
    b_est_list(i) = b_est;
    k_est_list(i) = k_est;

end

%%
figure(1);
plot(t(1:end-1), m_est_list, 'LineWidth', 1.5); 
hold on; 
yline(m, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Mass [kg]');
title('Mass Estimation');
legend('Estimated Mass (kg)', 'Real Mass (kg)');

figure(2);
plot(t(1:end-1), k_est_list, 'LineWidth', 1.5); 
hold on; 
yline(k, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Stiffness');
title('Stiffness Estimation');
legend('Estimated Stiffness', 'Real Stiffness');

figure(3);
plot(t(1:end-1), b_est_list, 'LineWidth', 1.5); 
hold on; 
yline(b, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Damping');
title('Damping Estimation');
legend('Estimated Damping', 'Real Damping');
