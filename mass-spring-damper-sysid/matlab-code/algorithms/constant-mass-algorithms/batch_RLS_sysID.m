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

end

% Batch least squares
theta = pinv(Y) * X;

% Recover physical parameters
m_est = Ts / theta(3);
b_est = (1 - theta(2)) * m_est / Ts;
k_est = -theta(1) * m_est / Ts;

fprintf('m estimation: %.3f\n', m_est)
fprintf('m real: %.1f\n', m)

fprintf('b estimation: %.3f\n', b_est)
fprintf('b real: %.1f\n', b)

fprintf('k estimation: %.3f\n', k_est)
fprintf('k real: %.1f\n', k)
