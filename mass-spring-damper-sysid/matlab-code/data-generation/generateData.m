clear; clc;

%% input type selection
% create_type = 1; % chirp signal
% create_type = 2; % sine wave
create_type = 3; % square wave
% create_type = 4; % step

%% Simülasyon Parametreleri
Ts = 0.01;       % Örnekleme zamanı [s]
T_end = 60;      % Simülasyon süresi [s]
rng(6);          % Tekrarlanabilir rastgele gürültü için seed

%% Sistem Parametreleri (Kütle-Yay-Sönüm)
k = 2;  
m = 3; 
b = 2;

%% Sürekli Zaman Matrisleri (CT)
A_CT = [0       1;
        -k/m   -b/m];
B_CT = [0;
        1/m];
C_CT = [1 0];    % Çıkış: Konum (Position)
D_CT = 0;

%% Sürekli Zamandan Ayrık Zamana Dönüşüm (ZOH)
sys_CT = ss(A_CT, B_CT, C_CT, D_CT);
sys_DT = c2d(sys_CT, Ts, 'zoh');
A_DT = sys_DT.A;
B_DT = sys_DT.B;
C_DT = sys_DT.C;
D_DT = sys_DT.D;

%% Gürültü Kovaryansları
sigma_F = 0.5;      % Süreç gürültüsü standart sapması (Örn: 0, 0.5, 5)
sigma_y = 0.02;     % Ölçüm gürültüsü standart sapması (Örn: 0, 0.02, 0.2)

%% Zaman Vektörü ve Giriş Sinyali
t = 0:Ts:T_end;
N = length(t);

%%
switch create_type
    case 1
        f0 = 0.1;
        f1 = 2;
        amplitude = 1;
        u = 1 + amplitude * chirp(t, f0, t(end), f1);
    case 2
        frequency = 0.5; % Sinüs dalgasının frekansı [Hz]
        amplitude = 1;   % Genlik
        bias = 1;        % Dikey kaydırma (ofset)
        u = bias + amplitude * sin(2 * pi * frequency * t);
    case 3
        frequency = 2; % Kare dalga frekansı [Hz]
        amplitude = 1;   % Genlik
        u = 1 + amplitude * square(2 * pi * frequency * t);
    case 4
        u = ones(1,N);
end

%% Bellek Ayırma (Veri Depolama)
x = zeros(2, N);              % Gerçek sistem durumları [konum; hız]
x(:,1) = [0; 0];              % Başlangıç koşulu
y = zeros(1, N);              % Çıkış sinyali
y(1) = C_DT * x(:,1) + sigma_y * randn;

%% Simülasyon Döngüsü (Veri Üretimi)
for i = 1:N-1
    force_noise = sigma_F * randn;
    
    % Gerçek sistem dinamikleri güncelleme
    x(:,i+1) = A_DT * x(:,i) + B_DT * (u(i) + force_noise);
    
    % Ölçüm
    y(i+1) = C_DT * x(:,i+1) + sigma_y * randn;
end

%% save data

% switch create_type
%     case 1
%         save testData_chirp_input u y k m b sigma_F sigma_y x t Ts T_end
%     case 2
%         save testData_sine_wave_input u y k m b sigma_F sigma_y x t Ts T_end
%     case 3
%         save testData_square_input.mat u y k m b sigma_F sigma_y x t Ts T_end
%     case 4
%         save testData_step_input.mat u y k m b sigma_F sigma_y x t Ts T_end
% end

%% plot PSD of the input

[PSD, f] = pwelch(u, [], [], [], 1/Ts);

figure(1);
plot(f, PSD)
xlabel('Frequency')
ylabel('Power Spectral Density')
grid on; hold on;