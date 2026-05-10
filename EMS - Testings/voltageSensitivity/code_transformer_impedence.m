clc; clear;

%% ================================
%% 1️⃣ FEEDER PARAMETERS
%% ================================

r_per_km = 0.443;

dist_m = [23.35 20.42 20.09 21.74 23.76 24.06 22 21.4 ...
          32.07 24.19 13.57 21.79 28.15 19.78];

dist_km = dist_m/1000;
R = r_per_km * dist_km;        % line resistances (Ohm)

c = cumsum(R);                 % cumulative resistance
n = 14;

%% ================================
%% 2️⃣ BUILD SENSITIVITY MATRIX
%% ================================

S = zeros(n);

for i = 1:n
    for j = 1:n
        S(i,j) = c(min(i,j));
    end
end

%% ================================
%% 3️⃣ TRANSFORMER UPSTREAM IMPEDANCE
%% ================================

% Example transformer:
% 100 kVA, 400V, 4% impedance

V_base = 400;        % LV base voltage (Volts)
S_base = 100e3;      % 100 kVA

Z_base = V_base^2 / S_base;

Z_tr_pu = 0.04;              % 4% transformer impedance
Z_tr = Z_tr_pu * Z_base;     % Ohm

% Assume R portion ~ 30% of Z (typical)
R_source = 0.3 * Z_tr;       % transformer resistance in Ohm

%% ================================
%% 4️⃣ READ EXCEL DATA
%% ================================

data = readtable('example_14_pole_load_solar_real.xlsx');

timestamps = data{:,1};   % safer than data.Timestamp

P_load = data{:,2:15};    
P_solar = data{:,16:29};

P_net = P_load - P_solar;  % kW

%% ================================
%% 5️⃣ PER UNIT CONVERSION
%% ================================

S_pu = S / Z_base;                    % feeder matrix in pu
R_source_pu = R_source / Z_base;      % transformer resistance in pu

P_pu = P_net / (S_base/1000);         % kW → pu

%% ================================
%% 6️⃣ VOLTAGE CALCULATION
%% ================================

% V_pu = zeros(size(P_pu));

% for t = 1:size(P_pu,1)
    
%     P_feeder = sum(P_pu(t,:));   % total feeder power
%     disp([': Total feeder power (pu) = ' num2str(P_feeder)])
%     % Full voltage model:
%     V_pu(t,:) = 1 ...
%         - (S_pu * P_pu(t,:)')' ...              % feeder drop
%         - R_source_pu * P_feeder;               % transformer drop
% end











for t = 1:size(P_net,1)
    
    V = zeros(1,n);
    V(1) = 1;  % slack pu
    
    % Convert P to actual watts
    P_actual = P_net(t,:)*1000;   % kW → W
    
    % Compute branch currents
    I = zeros(1,n);
    for k = 1:n
        P_downstream = sum(P_actual(k:n));
        I(k) = P_downstream / V_base;  % I = P/V
    end
    
    % Compute voltages along feeder
    for k = 1:n-1
        V(k+1) = V(k) - (R(k) * I(k)) / V_base;
    end
    
    V_pu(t,:) = V;
end

%% ================================
%% 7️⃣ DISPLAY RESULTS
%% ================================

disp('Voltage range (pu):')
disp([min(V_pu(:)) max(V_pu(:))])

%% ================================
%% 8️⃣ PLOT
%% ================================

figure;
plot(V_pu)
xlabel('Time Step')
ylabel('Voltage (pu)')
title('Voltage Profile of 14-Pole Feeder')
grid on

%% ================================
%% 9️⃣ EXPORT TO EXCEL
%% ================================

output_table = array2table(V_pu);

output_table = addvars(output_table, timestamps, ...
    'Before', 1, 'NewVariableNames','Timestamp');

output_table.Properties.VariableNames = ...
    [{'Timestamp'}, ...
    arrayfun(@(x) sprintf('Voltage_Pole_%d_pu',x),1:14,'UniformOutput',false)];

writetable(output_table, 'voltage_output.xlsx');

disp('Voltage results exported to voltage_output.xlsx');