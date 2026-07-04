\
clc; clear; close all;

%% =========================================
% SETTINGS
%% =========================================
emsFile   = 'MPC_Results_Fixed_Fixed_Fixed.xlsx';
emsSheet  = 'Pole_Scheduled_Solar';
masterFile = 'Master_XFMR_OtherFeeders_HeadMatch_RealBattery.dss';

Np = 14;
dt_hr = 0.25;

% Real battery parameters from user's image
P_batt_max = 50;    % kW
E_batt_kWh = 100;   % kWh
eta_ch     = 0.95;
eta_dis    = 0.95;
SOC_min    = 20;    % %
SOC_max    = 80;    % %
SOC0       = 50;    % %

PF_load = 0.95;
PF_pv   = 1.0;

other3_load_peak  = 120;   % kW
other3_solar_peak = 150;   % kW

%% =========================================
% READ EMS SHEET
%% =========================================
T = readtable(emsFile, 'Sheet', emsSheet, 'VariableNamingRule', 'preserve');
Nt = height(T);

Pload      = T{:, 2:15};     % B:O
Psolar     = T{:, 16:29};    % P:AC
Pimport_ref = T{:, 30};      % AD
Pexport_ref = T{:, 31};      % AE
Pbatt_sched = T{:, 32};      % AF, positive = charging, negative = discharging

F2_load_total  = sum(Pload, 2);
F2_solar_total = sum(Psolar, 2);

% Build normalized profiles for the lumped other 3 feeders
if max(F2_load_total) > 0
    other3_load  = other3_load_peak  * (F2_load_total  / max(F2_load_total));
else
    other3_load = zeros(Nt,1);
end

if max(F2_solar_total) > 0
    other3_solar = other3_solar_peak * (F2_solar_total / max(F2_solar_total));
else
    other3_solar = zeros(Nt,1);
end

%% =========================================
% COMPUTE SOC WITH REAL BATTERY LIMITS
%% =========================================
SOC = zeros(Nt,1);
SOC(1) = SOC0;

Pbatt_applied = zeros(Nt,1);

for t = 1:Nt
    Pb = Pbatt_sched(t);  % EMS sign: + charge, - discharge

    % Enforce power limits
    if Pb > 0
        Pb = min(Pb, P_batt_max);
    elseif Pb < 0
        Pb = -min(abs(Pb), P_batt_max);
    end

    % Enforce SOC limits using current SOC
    if SOC(t) >= SOC_max && Pb > 0
        Pb = 0;
    end
    if SOC(t) <= SOC_min && Pb < 0
        Pb = 0;
    end

    Pbatt_applied(t) = Pb;

    if t < Nt
        Pch  = max(Pb, 0);
        Pdis = max(-Pb, 0);

        SOC(t+1) = SOC(t) ...
            + (eta_ch * Pch * dt_hr / E_batt_kWh) * 100 ...
            - (Pdis * dt_hr / (eta_dis * E_batt_kWh)) * 100;

        SOC(t+1) = min(max(SOC(t+1), SOC_min), SOC_max);
    end
end

%% =========================================
% START OPENDSS
%% =========================================
DSSObj = actxserver('OpenDSSEngine.DSS');
ok = DSSObj.Start(0);
assert(ok == 1, 'OpenDSS failed to start.');

DSSText = DSSObj.Text;
DSSText.Command = 'Clear';
DSSText.Command = ['Compile "', fullfile(pwd, masterFile), '"'];

DSSCircuit  = DSSObj.ActiveCircuit;
DSSSolution = DSSCircuit.Solution;
assert(~isempty(DSSCircuit), 'No active circuit after compile.');

%% =========================================
% RESULT STORAGE
%% =========================================
Vpole        = nan(Nt, Np);
Vmax         = nan(Nt, 1);
Vmin         = nan(Nt, 1);
Phead_calc   = nan(Nt, 1);
Pimport_calc = nan(Nt, 1);
Pexport_calc = nan(Nt, 1);
Converged    = false(Nt, 1);

%% =========================================
% TIME SERIES REPLAY
%% =========================================
for t = 1:Nt
    % Update feeder-2 pole loads and PV
    for p = 1:Np
        DSSText.Command = sprintf('Edit Load.P%d kW=%.9f PF=%.4f', p, Pload(t,p), PF_load);
        DSSText.Command = sprintf('Edit Generator.GPV%d kW=%.9f PF=%.4f', p, Psolar(t,p), PF_pv);
    end

    % Update lumped other 3 feeders at LVBus (upstream of Line.Head)
    DSSText.Command = sprintf('Edit Load.Other3Feeders kW=%.9f PF=%.4f', other3_load(t), PF_load);
    DSSText.Command = sprintf('Edit Generator.Other3Solar kW=%.9f PF=1', other3_solar(t));

    % Update real battery with limits + efficiencies
    Pb = Pbatt_applied(t);  % positive charge, negative discharge in EMS convention

    if abs(Pb) < 1e-8
        DSSText.Command = sprintf('Edit Storage.BESS %%stored=%.6f state=idling kW=0 PF=1', SOC(t));
    elseif Pb > 0
        % charge in EMS => negative kW in OpenDSS Storage
        DSSText.Command = sprintf('Edit Storage.BESS %%stored=%.6f state=charging kW=%.9f PF=1', SOC(t), -Pb);
    else
        % discharge in EMS => positive kW in OpenDSS Storage
        DSSText.Command = sprintf('Edit Storage.BESS %%stored=%.6f state=discharging kW=%.9f PF=1', SOC(t), abs(Pb));
    end

    % Solve one snapshot
    DSSText.Command = 'Set Mode=Snapshot';
    DSSSolution.Solve;
    Converged(t) = DSSSolution.Converged ~= 0;

    if ~Converged(t)
        warning('Did not converge at step %d', t);
        continue;
    end

    % Voltages
    allNodes = string(DSSCircuit.AllNodeNames);
    allVpu   = DSSCircuit.AllBusVmagPu;
    Vmax(t)  = max(allVpu);
    Vmin(t)  = min(allVpu);

    for p = 1:Np
        busPrefix = lower("pole" + p + ".");
        idx = startsWith(lower(allNodes), busPrefix);
        if any(idx)
            Vpole(t,p) = max(allVpu(idx));
        end
    end

    % Feeder-2 head power only (Line.Head sending end)
    okElem = DSSCircuit.SetActiveElement('Line.Head');
    assert(okElem ~= 0, 'Could not activate Line.Head');
    pw = DSSCircuit.ActiveCktElement.Powers;

    Psend = pw(1) + pw(3) + pw(5);
    Phead_calc(t)   = Psend;
    Pimport_calc(t) = max(Psend, 0);
    Pexport_calc(t) = max(-Psend, 0);
end

%% =========================================
% ERROR METRICS
%% =========================================
imp_err = Pimport_calc - Pimport_ref;
exp_err = Pexport_calc - Pexport_ref;

fprintf('\n================ REPLAY SUMMARY ================\n');
fprintf('Time steps                : %d\n', Nt);
fprintf('Converged steps           : %d\n', nnz(Converged));
fprintf('Global max voltage        : %.5f pu\n', max(Vpole(:), [], 'omitnan'));
fprintf('Global min pole voltage   : %.5f pu\n', min(Vpole(:), [], 'omitnan'));
fprintf('Voltage violations >1.05  : %d\n', nnz(Vpole > 1.05));
fprintf('Mean |Import error|       : %.4f kW\n', mean(abs(imp_err), 'omitnan'));
fprintf('Max  |Import error|       : %.4f kW\n', max(abs(imp_err), [], 'omitnan'));
fprintf('Mean |Export error|       : %.4f kW\n', mean(abs(exp_err), 'omitnan'));
fprintf('Max  |Export error|       : %.4f kW\n', max(abs(exp_err), [], 'omitnan'));
fprintf('===============================================\n');

%% Save results
Tout = table((1:Nt).', SOC, Pbatt_sched, Pbatt_applied, other3_load, other3_solar, ...
             Pimport_ref, Pimport_calc, Pexport_ref, Pexport_calc, Vmax, Vmin, ...
    'VariableNames', {'t','SOC_percent','P_battery_sched_kW','P_battery_applied_kW', ...
    'Other3_Load_kW','Other3_Solar_kW','P_import_ref_kW','P_import_calc_kW', ...
    'P_export_ref_kW','P_export_calc_kW','Vmax_pu','Vmin_pu'});

for p = 1:Np
    Tout.(sprintf('Pole_%d_Vpu', p)) = Vpole(:,p);
end

writetable(Tout, 'OpenDSS_Replay_HeadMatch_RealBattery.csv');
fprintf('Saved detailed results to: OpenDSS_Replay_HeadMatch_RealBattery.csv\n');

%% Plots
figure('Name','Pole Voltages','Color','w');
plot(Vpole, 'LineWidth', 1.0);
hold on;
yline(1.05, '--r', '1.05 pu');
grid on;
xlabel('15-minute interval');
ylabel('Voltage (pu)');
title('Pole Voltages from OpenDSS Replay');

figure('Name','Feeder2 Head Import Export','Color','w');
plot(Pimport_ref, 'b', 'LineWidth', 1.4); hold on;
plot(Pimport_calc, '--r', 'LineWidth', 1.2);
plot(Pexport_ref, 'Color', [0.85 0.65 0], 'LineWidth', 1.4);
plot(Pexport_calc, '--m', 'LineWidth', 1.2);
grid on;
xlabel('15-minute interval');
ylabel('Power (kW)');
title('Feeder-2 Head Import/Export: Reference vs OpenDSS');
legend('Import ref','Import calc','Export ref','Export calc','Location','best');
