%% PARAMETERS
 r_per_km = 0.443;
%  r_per_km = 0.641;
dist_m = [23.35 20.42 20.09 21.74 23.76 24.06 22 21.4 ...
          32.07 24.19 13.57 21.79 28.15 19.78];

dist_km = dist_m/1000;
R = r_per_km * dist_km;

c = cumsum(R);     % cumulative resistance
n = 14;

%% Build Sensitivity Matrix
S = zeros(n);

for i = 1:n
    for j = 1:n
        S(i,j) = c(min(i,j));
    end
end

disp(S)

S=S*3; % Adjust sensitivity factor as needed


%% READ EXCEL DATA
data = readtable('Reconstructed_14Pole_Load_Solar.xlsx'); % real values excel

P_load = data{:,2:15};     % adjust if needed
P_solar = data{:,16:29};

P_net = P_load - P_solar;  % kW

%% Convert to per-unit
V_base = 400;       % example (change to yours) V
S_base = 100e3;       % example base power kVA

Z_base = V_base^2 / S_base;

S_pu = S / Z_base;

P_pu = P_net / (S_base/1000);  % kW → pu

%% Voltage Calculation
V_pu = zeros(size(P_pu));

for t = 1:size(P_pu,1)
    V_pu(t,:) = 1 - S_pu * P_pu(t,:)';
end


% %% Convert to per-unit
% V_base = 400;       % example (change to yours)
% S_base = 100e3;       % example base power

% Z_base = V_base^2 / S_base;

% % S_pu = S / Z_base;

% % P_pu = P_net / (S_base/1000);  % kW → pu

% %% Voltage Calculation
% V_pu = zeros(size(P_net));

% for t = 1:size(P_net,1)
%     V_pu(t,:) = 400 - S * P_net(t,:)';
% end


disp(V_pu)

% plot(V_pu(:,1:14))
% xlabel('Time Step')
% ylabel('Voltage (pu)')
% title('Voltage at Pole 14')


figure
plot(V_pu(:,1:14))
xlabel('Time Step')
ylabel('Voltage (pu)')
title('Voltage at Poles (1–14)')

% Limit y-axis
ylim([0.9 1.1])      % change if you need a different range

hold on

% Draw voltage limit lines
yline(0.95,'--','Low Voltage (0.95 pu)','LabelHorizontalAlignment','left')
yline(1.05,'--','High Voltage (1.05 pu)','LabelHorizontalAlignment','left')

hold off


% Combine timestamp + voltages
output_table = array2table(V_pu);
output_table = addvars(output_table, data.Timestamp, 'Before', 1);

% Rename columns
output_table.Properties.VariableNames = ...
    [{'Timestamp'}, ...
    arrayfun(@(x) sprintf('Voltage_Pole_%d_pu',x),1:14,'UniformOutput',false)];

% Write to Excel
writetable(output_table, 'voltage_output.xlsx');