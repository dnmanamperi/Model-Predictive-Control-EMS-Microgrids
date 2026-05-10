%% =========================================================
% TWO-LAYER CONSTANT-HORIZON MPC ENERGY MANAGEMENT SYSTEM
% 48h simulation, 24h rolling optimization horizon
% UPPER LAYER: tariff optimization
% LOWER LAYER: pole-wise MPC with battery + curtailment


% rule base voltage part eka add kara

%% =========================================================
clc; clear; close all;
rng(40);

%% =========================================================
%% BASIC SETTINGS
%% =========================================================
dt  = 0.25;               % 15 min
Nday = 96;                % 24h horizon
Nsim = 192;               % 48h data (needed for rolling)

Npoles = 14;
t = (0:Nsim-1)*dt;

%% =========================================================
%% READ REAL LOAD & PV DATA FROM EXCEL
%% =========================================================

data = readtable('afterforecasting.xlsx');

time_excel = data{:,1};   % First column = timestamp
timestamps = data{:,1};

Load = data{:,2:15}';     % 14 poles load (transpose!)
PV   = data{:,16:29}';

P_load =Load';
P_solar = PV';  % 14 poles solar (transpose!)

Load_tot = sum(Load);
PV_tot   = sum(PV);

%% =========================================================
%% POLE-WISE LOAD & PV PROFILES
%% =========================================================
time = (0:length(Load(1,:))-1)*dt;

figure('Name','Pole-wise Load & PV Profiles','Position',[100 50 1200 900])
for p = 1:Npoles
    subplot(4,4,p)
    plot(time, Load(p,:), 'b','LineWidth',1.2); hold on
    plot(time, PV(p,:),   'r','LineWidth',1.2);
    title(['Pole ' num2str(p)])
    xlabel('Hour'); ylabel('kW'); grid on
    if p==1
        legend('Load','PV')
    end
end
sgtitle('Individual Pole Load and Solar Profiles')

%% =========================================================
%% BUILD SENSITIVITY MATRIX
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
S=S*3; % Adjust sensitivity factor as needed

%% =========================================================
%% PER-UNIT CONVERSION
%% =========================================================
V_base = 400;           % V (LL)
S_base = 100e3;         % 100 kVA base

Z_base = V_base^2 / S_base;
S_pu   = S / Z_base;

P_net = P_load - P_solar;            % kW
P_pu  = P_net / (S_base/1000);       % kW → pu

%% =========================================================
%% VOLTAGE CALCULATION (SENSITIVITY METHOD)
%% =========================================================
V_pu = zeros(size(P_pu));

for t = 1:size(P_pu,1)
    V_pu(t,:) = 1 - S_pu * P_pu(t,:)';
end

V_poles = V_pu';     % make poles x time



%% =========================================================
%% VOLTAGE GRAPH (SENSITIVITY METHOD)
%% =========================================================

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

%% =========================================================
%% BATTERY
%% =========================================================
Ebat = 100;
Pbat_max = 20;
SOCmin = 0.2*Ebat;
SOCmax = 0.8*Ebat;
SOC0   = 0.5*Ebat;

%% =========================================================
%% GRID + COSTS
%% =========================================================
Pgrid_max = 60;  % grid limit

c_deg  = 10;   % battery degradation
c_curt = 25;   % curtailment penalty

%% =========================================================
%% UPPER LAYER SETTINGS
%% =========================================================

V_ref = 1.0;                   % per unit reference voltage
k_voltage = 50;                % sensitivity for export tariff
Texp_base = 20;                % base export price
Timp_base = 30;                % base import price
delta = 0.2;                    % max deviation from base import

% Simulate voltages at each pole (random for demo)
% V_poles = 0.95 + 0.1*rand(Npoles,Nsim);

%% =========================================================
%% RESULT STORAGE
%% =========================================================
Pimp=zeros(Nday,1);
Pexp=zeros(Nday,1);
Pbat=zeros(Nday,1);
Curt=zeros(Nday,1);
SOC =zeros(Nday,1);

% ===== NEW: pole-wise storage =====
Pexp_pole = zeros(Npoles,Nday);
Curt_pole = zeros(Npoles,Nday);
Texp_pole = zeros(Npoles,Nday);

SOC_now = SOC0;
total_cost = 0;

options = optimoptions('linprog','Display','none');

% Timp_prev = Timp_base;   % initialize previous tariff

%% =========================================================
%% TRUE CONSTANT HORIZON MPC WITH UPPER LAYER + POLE-WISE
%% =========================================================
for k=1:Nday
    
    Nh = Nday;               % <<< CONSTANT horizon
    idx_start = k;
    idx_end   = k+Nh-1;
    
    %% =============================
    %% UPPER LAYER: LP TARIFF
    %% =============================
    
    T_imp = Timp_base;
    % T_exp = Texp_base;
    
    T_exp_vec = zeros(Npoles,1);
    
    for p = 1:Npoles
        
        Vn = V_poles(p,k);   % voltage at pole p, time k
        
        if Vn <= 0.95
            T_exp_vec(p) = 26;
            
        elseif Vn >= 1.05
            T_exp_vec(p) = 14;
            
        else
            T_exp_vec(p) = 26 - ((Vn - 0.95)/(1.05 - 0.95))*(26 - 14);
        end
        
    end
    
    %% =============================
    %% LOWER LAYER MPC (pole-wise)
    %% =============================
    nv_per = 1 + Npoles + 2 + Npoles + 1; % [Pimp, Pexp_1..Npoles, Pch,Pdis,Curt_1..Npoles,SOC]
    nvar = Nh*nv_per;
    
    %% COST VECTOR
    f_mpc=zeros(nvar,1);
    for i=1:Nh
        idx=(i-1)*nv_per;
        f_mpc(idx+1)=T_imp*dt;                   % import
        f_mpc(idx+2:idx+1+Npoles)=-dt*T_exp_vec;     % export revenue
        f_mpc(idx+2+Npoles)=c_deg*dt;            % Pch degradation
        f_mpc(idx+3+Npoles)=c_deg*dt;            % Pdis degradation
        f_mpc(idx+4+Npoles:idx+3+2*Npoles)=c_curt*dt; % Curtailment
    end
    
    
    %% BOUNDS
    lb=zeros(nvar,1); ub=inf(nvar,1);
    for i=1:Nh
        idx=(i-1)*nv_per;
        % Pimp
        ub(idx+1)=Pgrid_max;
        % Pexp
        ub(idx+2:idx+1+Npoles)=PV(:,idx_start+i-1);
        % Pch,Pdis
        ub(idx+2+Npoles)=Pbat_max;
        ub(idx+3+Npoles)=Pbat_max;
        % Curtailment
        ub(idx+4+Npoles:idx+3+2*Npoles)=0.5*PV(:,idx_start+i-1);
        % SOC
        lb(idx+4+2*Npoles)=SOCmin;
        ub(idx+4+2*Npoles)=SOCmax;
    end
    
    %% EQUALITY CONSTRAINTS
    Aeq=[]; beq=[];
    
    % Power balance
    for i=1:Nh
        row=zeros(1,nvar);
        idx=(i-1)*nv_per;
        row(idx+1)=1; % Pimp
        row(idx+2:idx+1+Npoles)=-1; % Pexp
        row(idx+2+Npoles)=1;  % Pch
        row(idx+3+Npoles)=-1; % Pdis
        row(idx+4+Npoles:idx+3+2*Npoles)=-1; % Curtailment
        beq_i = sum(Load(:,idx_start+i-1)) - sum(PV(:,idx_start+i-1));
        Aeq=[Aeq;row]; beq=[beq; beq_i];
    end
    
    % SOC dynamics
    for i=1:Nh-1
        row=zeros(1,nvar);
        idx=(i-1)*nv_per; idx2=i*nv_per;
        row(idx+4+2*Npoles)=1;          % SOC(h)
        row(idx+2+Npoles)=-dt;          % Pch
        row(idx+3+Npoles)= dt;          % Pdis
        row(idx2+4+2*Npoles)=-1;        % SOC(h+1)
        Aeq=[Aeq;row]; beq=[beq;0];
    end
    
    % Initial SOC measured
    row=zeros(1,nvar);
    row(4+2*Npoles)=1;
    Aeq=[Aeq;row]; beq=[beq;SOC_now];
    
    %% INEQUALITY: Grid limit
    A=[]; b=[];
    for i=1:Nh
        idx=(i-1)*nv_per;
        row1=zeros(1,nvar); row1(idx+1)=1; row1(idx+2:idx+1+Npoles)=-1;
        row2=zeros(1,nvar); row2(idx+1)=-1; row2(idx+2:idx+1+Npoles)=1;
        A=[A;row1;row2];
        b=[b;Pgrid_max;Pgrid_max/3];
    end
    
    %% SOLVE MPC
    x = linprog(f_mpc,A,b,Aeq,beq,lb,ub,options);
    if isempty(x)
        warning('Optimization failed at step %d', k); break
    end
    x = reshape(x,nv_per,Nh)';
    
    %% APPLY FIRST STEP
    
    Pimp(k)=x(1,1);
    
    % -------- POLE EXPORTS ----------
    Pexp_pole(:,k)=x(1,2:1+Npoles)';     % store each pole
    Pexp(k)=sum(Pexp_pole(:,k));
    
    % -------- CURTAILMENT ----------
    Curt_pole(:,k)=x(1,4+Npoles:3+2*Npoles)';
    Curt(k)=sum(Curt_pole(:,k));
    
    % -------- BATTERY ----------
    Pbat_k = x(1,3+Npoles)-x(1,2+Npoles);
    Pbat(k)=Pbat_k;
    
    SOC_now = SOC_now + dt*(x(1,3+Npoles)-x(1,2+Npoles));
    SOC(k)=SOC_now;
    
    % -------- TARIFF ----------
    Texp_pole(:,k)=T_exp_vec;
    
    total_cost = total_cost + f_mpc(1:nv_per)'*x(1,:)';
    
end

SOC_pct = 100*SOC/Ebat;
disp(['Total daily cost (TRUE MPC) = ',num2str(total_cost)])

%% =========================================================
%% PLOTS
%% =========================================================
figure

% time_plot = time(1:Nday);  % trim time to first 24h
% subplot(5,1,1)
% plot(time_plot, Load_tot(1:Nday), 'k', time_plot, PV_tot(1:Nday), 'g', 'LineWidth',1.4)
% legend('Load','PV')
% title('Load & PV')
% grid on


time_plot = 1:Nday;   % 1 to 96

subplot(5,1,1)
plot(time_plot, Load_tot(1:Nday), 'k', time_plot, PV_tot(1:Nday), 'g','LineWidth',1.4)

xlabel('Time Step (15 min)')
ylabel('kW')
xlim([1 100])
grid on
legend('Load','PV')
title('Load & PV')

subplot(5,1,2)
plot(Pimp,'b'); hold on; plot(Pexp,'r')
yline(Pgrid_max,'--k')
title('Grid Power'); grid on

subplot(5,1,3)
plot(Pbat); yline(0)
title('Battery Power'); grid on

subplot(5,1,4)
plot(SOC_pct); ylim([0 100])
title('SOC (%)'); grid on

subplot(5,1,5)
plot(Curt)
title('Curtailment'); grid on
xlabel('Step')

%% =========================================================
%% EXPORT TO EXCEL (SYSTEM + VOLTAGE + ENERGY FLOWS)
%% =========================================================

% SystemTable = table;

% SystemTable.Time = time_excel(1:Nday);

% % System power flows
% SystemTable.Load_total      = Load_tot(1:Nday)';
% SystemTable.Solar_total     = PV_tot(1:Nday)';
% SystemTable.Curtailment     = Curt;

% % Grid and battery
% SystemTable.P_import        = Pimp;
% SystemTable.P_export        = Pexp;
% SystemTable.P_battery       = Pbat;
% SystemTable.SOC_percent     = SOC_pct;


% writetable(SystemTable,'MPC_Results_Rule-Based_PoleWise.xlsx');

% disp('Results exported to MPC_Results_Rule-Based_PoleWise.xlsx')









%% =========================================================
%% EXPORT POLE-WISE NET INJECTION (USING Pexp_pole)
%% =========================================================


%% =========================================================
%% POLE-WISE NET INJECTION CALCULATION
%% =========================================================

Pnet_pole = zeros(Npoles,Nday);

for k = 1:Nday
    for p = 1:Npoles
        
        % Optimized net injection
        Pnet_pole(p,k) = PV(p,k)...
                        - Load(p,k) ...
                        - Curt_pole(p,k);
        
        % This should equal Pexp_pole(p,k)
    end
end



filename = 'MPC_Results_Rule-Based_PoleWise.xlsx';

% --- System level ---
SystemTable = table;

SystemTable.Time = time_excel(1:Nday);
SystemTable.Load_total  = Load_tot(1:Nday)';
SystemTable.Solar_total = PV_tot(1:Nday)';
SystemTable.Curtailment = Curt;
SystemTable.P_import    = Pimp;
SystemTable.P_export    = Pexp;
SystemTable.P_battery   = Pbat;
SystemTable.SOC_percent = SOC_pct;

writetable(SystemTable,filename,'Sheet','System');

% --- Pole-wise Net Injection (directly from export variable) ---
PoleNetTable = array2table(Pnet_pole');
PoleNetTable.Time = time_excel(1:Nday);

% Rename columns nicely
for p = 1:Npoles
    PoleNetTable.Properties.VariableNames{p} = ...
        ['Pole_' num2str(p) '_Injection_kW'];
end

writetable(PoleNetTable,filename,'Sheet','Pole_Net_Injection');

disp('Pole-wise net injection exported successfully.')









%% =========================================================
%% PERFORMANCE METRICS
%% =========================================================
Pgrid = Pimp - Pexp;

E_import = sum(Pimp)*dt;
E_export = sum(Pexp)*dt;
E_curt   = sum(Curt)*dt;
E_pv     = sum(PV_tot(1:length(Pimp)))*dt;

E_bat_throughput = sum(abs(Pbat))*dt;

P_peak = max(abs(Pgrid));
P_rms  = rms(Pgrid);

PV_utilization = 1 - E_curt/E_pv;

SOC_var = var(SOC_pct);

fprintf('\n============= PERFORMANCE METRICS =============\n');
fprintf('Total cost             = %.2f LKR\n', total_cost);
fprintf('Import energy          = %.2f kWh\n', E_import);
fprintf('Export energy          = %.2f kWh\n', E_export);
fprintf('Curtailment energy     = %.2f kWh\n', E_curt);
fprintf('PV utilization         = %.2f %%\n', 100*PV_utilization);
fprintf('Battery throughput     = %.2f kWh\n', E_bat_throughput);
fprintf('Peak grid power        = %.2f kW\n', P_peak);
fprintf('RMS grid power         = %.2f kW\n', P_rms);
fprintf('SOC variance           = %.2f\n', SOC_var);
fprintf('===============================================\n');



%% =========================================================
%% POLE-WISE CURTAILMENT
%% =========================================================
figure('Name','Pole-wise Curtailment','Position',[100 50 1200 900])
for p = 1:Npoles
    subplot(4,4,p)
    % Assuming we stored Curt_pole during MPC (NxNpoles)
    if exist('Curt_pole','var')
        plot(time_plot, Curt_pole(p,1:Nday),'r','LineWidth',1.2);
    else
        plot(time_plot, zeros(1,Nday),'r','LineWidth',1.2); % placeholder
    end
    title(['Pole ' num2str(p)])
    xlabel('Hour'); ylabel('kW'); grid on
end
sgtitle('Pole-wise Curtailment')

%% =========================================================
%% POLE-WISE EXPORT POWER
%% =========================================================
figure('Name','Pole-wise Export Power','Position',[100 50 1200 900])
for p = 1:Npoles
    subplot(4,4,p)
    if exist('Pexp_pole','var')
        plot(time_plot, Pexp_pole(p,1:Nday),'b','LineWidth',1.2);
    else
        plot(time_plot, zeros(1,Nday),'b','LineWidth',1.2); % placeholder
    end
    title(['Pole ' num2str(p)])
    xlabel('Hour'); ylabel('kW'); grid on
end
sgtitle('Pole-wise Export Power')

%% =========================================================
%% POLE-WISE EXPORT TARIFFS
%% =========================================================
figure('Name','Pole-wise Export Tariff','Position',[100 50 1200 900])
for p = 1:Npoles
    subplot(4,4,p)
    if exist('Texp_pole','var')
        plot(time_plot, Texp_pole(p,1:Nday),'m','LineWidth',1.2);
    else
        plot(time_plot, Texp_voltage(p)*ones(1,Nday),'m','LineWidth',1.2); % default voltage-based
    end
    title(['Pole ' num2str(p)])
    xlabel('Hour'); ylabel('LKR/kWh'); grid on
end
sgtitle('Pole-wise Export Tariffs')


%% =========================================================
%% POLE-WISE VOLTAGE PROFILES
%% =========================================================
time_plot = (0:Nday-1)*dt;

figure('Name','Pole-wise Voltage Profiles','Position',[100 50 1200 900])

for p = 1:Npoles
    subplot(4,4,p)
    plot(time_plot, V_poles(p,1:Nday),'LineWidth',1.3)
    yline(V_ref,'--')    % reference line
    
    title(['Pole ' num2str(p)])
    xlabel('Hour')
    ylabel('p.u.')
    ylim([0.9 1.05])     % zoom for clarity
    grid on
end

sgtitle('Pole-wise Voltage (per-unit)')


%% =========================================================
%% VOLTAGE vs TARIFF (comparison)
%% =========================================================
figure('Name','Voltage vs Tariff (Pole 1 example)')

yyaxis left
plot(time_plot,V_poles(1,1:Nday),'LineWidth',1.4)
ylabel('Voltage (p.u.)')

yyaxis right
plot(time_plot,Texp_pole(1,:),'LineWidth',1.4)
ylabel('Export Tariff (LKR/kWh)')

xlabel('Hour')
title('Voltage–Tariff Relationship (Pole 1)')
grid on


%% =========================================================
%% POLE-WISE VOLTAGE PROFILES - sensitivity analysis
%% =========================================================
time_plot = (0:Nday-1)*dt;

figure('Name','Pole-wise Voltage Profiles','Position',[100 50 1200 900])

for p = 1:Npoles
    subplot(4,4,p)
    plot(time_plot, V_poles(p,1:Nday),'LineWidth',1.3)
    yline(V_ref,'--')    % reference line
    
    title(['Pole ' num2str(p)])
    xlabel('Hour')
    ylabel('p.u.')
    ylim([0.9 1.05])     % zoom for clarity
    grid on
end

sgtitle('Pole-wise Voltage (per-unit)')



%% =========================================================
%% SAVE RESULTS TO EXCEL
%% =========================================================

filename = 'dispatch_results.xlsx';

T = table;

T.Time = time_plot';
T.P_import = Pimp;
T.P_export_total = Pexp;
T.P_battery = Pbat;
T.SOC_percent = SOC_pct;
T.Curtailment_total = Curt;

writetable(T, filename, 'Sheet', 'Summary');

% Pole-wise export
Texp_table = array2table(Pexp_pole');
Texp_table.Time = time_plot';
writetable(Texp_table, filename, 'Sheet', 'Pole_Export');

% Pole-wise curtailment
Tcurt_table = array2table(Curt_pole');
Tcurt_table.Time = time_plot';
writetable(Tcurt_table, filename, 'Sheet', 'Pole_Curtailment');

disp('Results saved to dispatch_results.xlsx');