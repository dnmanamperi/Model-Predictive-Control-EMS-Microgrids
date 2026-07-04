data = readtable('Initial Details\Voltage_Tariff_Pole14.xlsx');

time = data.TimeStep;
V = data.Voltage_pu;

RBC = data.RBC;
OPT = data.OPT;
FIX = data.fixed;
set(0,'DefaultLineLineStyle','-')
figure('Name','Voltage vs Tariff Comparison')
set(gcf, 'Color', 'w');
% Left axis → Voltage vs Time
yyaxis left
plot(time, V, 'k', 'LineWidth',1.5)
ylim([0.90 1.15])
ylabel('Voltage (pu)')

% Right axis → Tariffs
yyaxis right
plot(time, RBC, '-b', 'LineWidth',1.5); hold on
plot(time, OPT, '-r', 'LineWidth',1.5)
plot(time, FIX, '-g', 'LineWidth',1.2)
ylim([12 20])
ylabel('Tariff (LKR/kWh)')

xlabel('Time Step (15 min)')


legend('Voltage','RBC Tariff','Optimization Tariff','Fixed Tariff')

title('Voltage and Tariff Comparison (Pole 14)')
