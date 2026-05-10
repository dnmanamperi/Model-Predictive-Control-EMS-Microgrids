%% =========================================================
% UPPER LAYER – 24h EXPORT LIMITS WITH 80% FAIRNESS
% Voltage-based export control (Overvoltage only)
%% =========================================================

clear; clc;

Npoles = 14;
Vslack = 1.0;           % pu
Vmax   = 1.05;          % pu

%% =========================================================
% READ EXCEL DATA
%% =========================================================
filename = 'afterforecasting.xlsx';
data = readmatrix(filename);

Load_all = data(:,1:14);      % per-pole load (kW)
PV_all   = data(:,15:28);     % per-pole available PV (kW)

Nsim = size(Load_all,1);

% Use first 24 rows if hourly
% Use first 96 rows if 15-min
Nsteps = min(96,Nsim);

Load_all = Load_all(1:Nsteps,:);
PV_all   = PV_all(1:Nsteps,:);

%% =========================================================
% INSTALLED PV CAPACITY (kW)
%% =========================================================
Installed = [12 6 0 0 5 0 5 0 16 12.5 0 4.5 0 11]';

%% =========================================================
% BUILD VOLTAGE SENSITIVITY MATRIX
%% =========================================================
r_per_km = 0.443;

dist_m = [23.35 20.42 20.09 21.74 23.76 24.06 22 21.4 ...
          32.07 24.19 13.57 21.79 28.15 19.78];

dist_km = dist_m/1000;
R = r_per_km * dist_km;
c = cumsum(R);

S = zeros(Npoles);
for i = 1:Npoles
    for j = 1:Npoles
        S(i,j) = c(min(i,j));
    end
end

% Convert to pu/kW
Vbase_val = 400;      % volts
Sbase     = 100e3;    % 100 kVA base
Zbase     = Vbase_val^2 / Sbase;
S = S / Zbase;

%% =========================================================
% STORAGE
%% =========================================================
Xmax = zeros(Nsteps,Npoles);
V_profile = zeros(Nsteps,Npoles);

options = optimoptions('linprog','Display','none');

%% =========================================================
% TIME LOOP
%% =========================================================
for t = 1:Nsteps
    
    Pload = Load_all(t,:)';
    PVavail = PV_all(t,:)';
    
    % Use minimum of forecast PV and installed
    PVcap = min(PVavail, Installed);
    
    %% ---- 80% Minimum Guarantee ----
    lb = 0.1 * Installed;
    ub = PVcap;
    
    % Poles without PV
    lb(Installed==0) = 0;
    ub(Installed==0) = 0;
    
    %% ---- Voltage base from load only ----
    V_base = Vslack + S*(-Pload);
    
    % Voltage constraint: V_base + S*x <= 1.05
    A = S;
    b = Vmax - V_base;
    
    % Objective: maximize total export
    f = -ones(Npoles,1);
    
    [x_opt,~,exitflag] = linprog(f,A,b,[],[],lb,ub,options);
    
    if exitflag ~= 1
        % If 80% infeasible, reduce to feasible proportional factor
        alpha = 0.8;
        feasible = false;
        
        while alpha > 0
            test_x = alpha * Installed;
            test_x = min(test_x, PVcap);
            
            Vtest = Vslack + S*(test_x - Pload);
            
            if all(Vtest <= Vmax)
                x_opt = test_x;
                feasible = true;
                break;
            end
            
            alpha = alpha - 0.05;   % reduce fairness gradually
        end
        
        if ~feasible
            x_opt = zeros(Npoles,1);
        end
    end
    
    Xmax(t,:) = x_opt';
    V_profile(t,:) = (Vslack + S*(x_opt - Pload))';
    
end

%% =========================================================
% OUTPUT
%% =========================================================
disp('24h Export Limits (kW):')
disp(Xmax)

%% =========================================================
% PLOTS
%% =========================================================

% 1️⃣ Total Hosting Capacity
figure;
plot(sum(Xmax,2),'LineWidth',1.5);
xlabel('Time Step');
ylabel('Total Export (kW)');
title('24h Total Export Capacity');
grid on;

% 2️⃣ Pole-wise Export Heatmap
figure;
imagesc(Xmax');
colorbar;
xlabel('Time Step');
ylabel('Pole Number');
title('Export Limits per Pole (kW)');

% 3️⃣ Maximum Voltage
figure;
plot(max(V_profile,[],2),'LineWidth',1.5);
hold on;
yline(1.05,'r--');
xlabel('Time Step');
ylabel('Max Voltage (pu)');
title('Maximum Feeder Voltage (24h)');
grid on;