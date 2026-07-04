%% =========================================================
clc; clear; close all;


%% =========================================================
%% BASIC SETTINGS
%% =========================================================
dt  = 0.25;               % 15 min
Nday = 96;                % 24h horizon
Nsim = 192;               % 48h data (needed for rolling)

Npoles = 14;
t = (0:Nsim-1)*dt;



%% =========================================================
%% READ ACTUAL LOAD & PV DATA FROM EXCEL
%% =========================================================

data1 = readtable('Reconstructed_14_Pole_Load_Solar_Real.xlsx'); % real values for next 48h (96 time steps)

time_excel = data1{:,1};   % First column = timestamp
timestamps = data1{:,1};

Load_real = data1{:,2:15}';     % 14 poles load (transpose!)
PV_real   = data1{:,16:29}';

P_load_real =Load_real';
P_solar_real = PV_real';  % 14 poles solar (transpose!)

Load_tot_real = sum(Load_real);
PV_tot_real   = sum(PV_real);

%% =========================================================
%% PLOTS
%% =========================================================

figure
time_plot = 1:Nday;   % 1 to 96 time steps for 24h horizon

% subplot(5,1,1)
% plot(time_plot, Load_tot(1:Nday), 'k', time_plot, PV_tot(1:Nday), 'g','LineWidth',1.4)
plot(time_plot, Load_tot_real(1:Nday), 'b', time_plot, PV_tot_real(1:Nday), 'r','LineWidth',1.4)
xlabel('Time Step (15 min)', 'FontSize', 12)
ylabel('Power (kW)', 'FontSize', 12)
xlim([0 100])
set(gcf, 'Color', 'w');
grid on
legend('Load','PV', 'FontSize', 12)
title('BK 28 Feeder - Total Load & PV - 2025-07-10') 


%% =========================================================
%% READ ACTUAL VOLTAGE SENSITIVITY MODEL ERROR FROM EXCEL
%% =========================================================

data1 = readtable('Initial Details\Sensitivity Matrix Error.xlsx','Sheet','Matlab'); % real values for next 48h (96 time steps)

time_excel = data1{:,1};   % First column = timestamp
timestamps = data1{:,1};

Voltage_real = data1{:,3};     % 14 poles load (transpose!)
VOltage_sensitivity   = data1{:,2};

%% =========================================================
%% PLOTS
%% =========================================================

figure
time_plot = 1:Nday;   % 1 to 96 time steps for 24h horizon

% subplot(5,1,1)
% plot(time_plot, Load_tot(1:Nday), 'k', time_plot, PV_tot(1:Nday), 'g','LineWidth',1.4)
plot(time_plot, Voltage_real(1:Nday), 'b', time_plot, VOltage_sensitivity(1:Nday), 'r','LineWidth',1.4)
xlabel('Time Step (15 min)', 'FontSize', 12)
ylabel('Voltage (pu)', 'FontSize', 12)
xlim([0 100])
set(gcf, 'Color', 'w');
grid on
legend('OpenDSS Power Flow Results','Sensitivity Model Results', 'FontSize', 12)
title('Pole - 14 Voltage Variations', 'FontSize', 12)