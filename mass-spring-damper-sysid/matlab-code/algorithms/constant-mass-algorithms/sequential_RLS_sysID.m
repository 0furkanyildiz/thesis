clear; clc

%% load data

load testData_chirp_input
% load testData_sine_wave_input.mat
% load testData_step_input.mat
% load testData_square_input.mat

%% simulation length

N = length(t);

%% P matrix

% P0 = diag([0.1, 0.1, 0.1]);
% P0 = diag([10, 10, 10]);
P0 = diag([100, 100, 100]);

P = P0;

%% initial estimate

theta_hat = zeros(3,1);

%% forgetting factor

lambda = 1;
% lambda = 0.98;

%% Discrete-time simulation using for loop
for i = 1:N-1

    phi = [x(1,i); x(2,i); u(i)]';

    y = x(2,i+1);

    K = (P*phi') / (lambda + phi*P*phi');

    theta_hat = theta_hat + K*(y - phi*theta_hat);

    P = (P - K*phi*P)/lambda;

    if abs(theta_hat(3)) > 1e-8
        m_est = Ts/theta_hat(3);
        b_est = (1-theta_hat(2))*m_est/Ts;
        k_est = -theta_hat(1)*m_est/Ts;
    else
        m_est = NaN;
        b_est = NaN;
        k_est = NaN;
    end

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
