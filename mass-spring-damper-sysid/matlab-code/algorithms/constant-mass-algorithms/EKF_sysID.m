clear; clc

%% load data

% load testData_chirp_input
% load testData_sine_wave_input.mat
% load testData_step_input.mat
load testData_square_input.mat

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

%% process noise covariance Q

Q = B_DT * sigma_F^2 * B_DT';

%% measurement noise covariance R

R = sigma_y^2;

%% simulation

t = 0:Ts:T_end;
N = length(t);

I = eye(3);

%% Kalman
% Initial Kalman filter estimate
x_hat = zeros(3,N);
x_hat(:,1) = [0; 0; 6];

P0 = diag([0.1, 0.1, 10]);
% P0 = diag([0.1, 0.1, 50]);
% P0 = diag([0.1, 0.1, 100]);

P = P0;

% Store Kalman gains
K_hist = zeros(3,N);

%% Discrete-time simulation using for loop
for i = 1:N-1

    m_hat = max(x_hat(3,i), 0.05);
    v_hat = x_hat(2, i);
    pos_hat = x_hat(1, i);

    A_ct_aug = [0             1          0;
            -k/m_hat     -b/m_hat   (k*pos_hat + b*v_hat-u(i))/m_hat^2;
            0             0          0];

    B_ct_aug = [0;
            1/m_hat;
            0];

    C_ct_aug = [1 0 0];    % Output: position
    D_ct_aug = 0;

    %%
    sys_CT_aug = ss(A_ct_aug, B_ct_aug, C_ct_aug, D_ct_aug);
    sys_DT_aug = c2d(sys_CT_aug, Ts, 'zoh');

    A_DT_aug = sys_DT_aug.A;
    B_DT_aug = sys_DT_aug.B;
    C_DT_aug = sys_DT_aug.C;
    D_DT_aug = sys_DT_aug.D;

    %%S
    A_est_CT = [0 1;
                -k/m_hat -b/m_hat];

    B_est_CT = [0; 1/m_hat];

    sys_est_CT = ss(A_est_CT, B_est_CT, C_CT, D_CT);

    sys_est_DT = c2d(sys_est_CT, Ts, 'zoh');
    A_est_DT = sys_est_DT.A;
    B_est_DT = sys_est_DT.B;

    %%
    Q_kalman = zeros(3);
    Q_kalman(1:2,1:2) = ...
        B_est_DT * sigma_F^2 * B_est_DT';
    Q_kalman(3,3) = 1e-6;

    % prediction
    x_pred = [A_est_DT*x_hat(1:2,i) + B_est_DT*u(i);
              x_hat(3,i)];

    % update error covariance matrix
    P_pred = A_DT_aug*P*A_DT_aug' + Q_kalman;

    % calculate kalman gain
    K = P_pred * C_DT_aug' / (C_DT_aug * P_pred * C_DT_aug' + R);

    % state estimate update
    innovation = y(i+1) - C_DT_aug*x_pred;
    x_hat(:,i+1) = x_pred + K*innovation;

    % joseph form
    P = (I-K*C_DT_aug)*P_pred*(I-K*C_DT_aug)' + K*R*K';

    % store kalman gain
    K_hist(:,i) = K;
end

figure(1);
plot(t, x_hat(1,:), 'b-', 'LineWidth', 1.5); 
hold on; 
plot(t, x(1,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('X [m]');
title('Position Estimation Using EKF');
legend('Estimated Position (m)', 'Real Position (m)');

figure(2);
plot(t, x_hat(2,:), 'b-', 'LineWidth', 1.5); 
hold on; 
plot(t, x(2,:), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocity Estimation Using EKF');
legend('Estimated Velocity (m/s)', 'Real Velocity (m/s)');

figure(3);
plot(t, x_hat(3,:), 'LineWidth', 1.5); 
hold on; 
yline(m, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Mass [kg]');
title('Mass Estimation Using EKF');
legend('Estimated Mass (kg)', 'Real Mass (kg)');

% legend('Estimated Mass (kg) - Low P0 (10)', 'Estimated Mass (kg) - Medium P0 (50)', 'Estimated Mass (kg) - High P0 (100)', 'Real Mass (kg)');