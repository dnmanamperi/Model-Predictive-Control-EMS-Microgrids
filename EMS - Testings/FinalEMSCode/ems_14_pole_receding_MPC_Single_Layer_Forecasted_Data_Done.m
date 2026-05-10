%% =========================================================
% TRUE CONSTANT-HORIZON MPC + VOLTAGE CALCULATION
% Forecast Driven (Excel Input)
% 14 Pole System
%% =========================================================

clc; clear; close all;

%% =========================================================
%% BASIC SETTINGS
%% =========================================================
dt  = 0.25;        % 15 min
Nday = 96;         % 24h horizon

Npoles = 14;

%% =========================================================
%% READ FORECAST DATA FROM EXCEL
%% =========================================================
data = readtable('afterforecasting.xlsx');

timestamps = data{:,1};

P_load  = data{:,2:15};     % 14 pole load (kW)
P_solar = data{:,16:29};    % 14 pole solar (kW)

Nsim = size(P_load,1);      % should be 192

Load = P_load';
PV   = P_solar';

Load_tot = sum(Load);
PV_tot   = sum(PV);

time48 = (0:Nsim-1)*dt;

%% =========================================================
%% POLE-WISE LOAD & PV PROFILES (48h)
%% =========================================================
figure('Name','Pole-wise Load & PV Profiles','Position',[100 50 1200 900])

for p = 1:Npoles
    subplot(4,4,p)
    plot(time48, Load(p,:), 'b','LineWidth',1.2); hold on
    plot(time48, PV(p,:),   'r','LineWidth',1.2);
    title(['Pole ' num2str(p)])
    xlabel('Hour')
    ylabel('kW')
    grid on
    if p==1
        legend('Load','PV')
    end
end
sgtitle('Individual Pole Load and Solar Profiles (48h)')


%% =========================================================
%% BATTERY
%% =========================================================
Ebat = 50;
Pbat_max = 20;

SOCmin = 0.2*Ebat;
SOCmax = 0.8*Ebat;
SOC0   = 0.5*Ebat;

%% =========================================================
%% GRID + COSTS
%% =========================================================
Pgrid_max = 50;

c_imp  = 30;
c_exp  = 25;
c_deg  = 15; %2
c_curt = 20; %1

%% =========================================================
%% RESULT STORAGE
%% =========================================================
Pimp=zeros(Nday,1);
Pexp=zeros(Nday,1);
Pbat=zeros(Nday,1);
Curt=zeros(Nday,1);
SOC =zeros(Nday,1);

SOC_now = SOC0;
total_cost = 0;

options = optimoptions('linprog','Display','none');

%% =========================================================
%% TRUE CONSTANT HORIZON MPC
%% =========================================================
for k=1:Nday
    
    Nh = Nday;
    idx_start = k;
    idx_end   = k+Nh-1;
    
    nvar = 6*Nh;

    %% COST
    f=zeros(nvar,1);
    for i=1:Nh
        idx=(i-1)*6;
        f(idx+1)=c_imp*dt;
        f(idx+2)=-c_exp*dt;
        f(idx+3)=c_deg*dt;
        f(idx+4)=c_deg*dt;
        f(idx+5)=c_curt*dt;
    end

    %% BOUNDS
    lb=zeros(nvar,1);
    ub=inf(nvar,1);

    for i=1:Nh
        idx=(i-1)*6;
        ub(idx+1)=250;
        ub(idx+2)=250;
        ub(idx+3)=Pbat_max;
        ub(idx+4)=Pbat_max;
        ub(idx+5)=PV_tot(idx_start+i-1);
        lb(idx+6)=SOCmin;
        ub(idx+6)=SOCmax;
    end

    %% EQUALITY CONSTRAINTS
    Aeq=[]; beq=[];

    % Power balance
    for i=1:Nh
        row=zeros(1,nvar);
        idx=(i-1)*6;

        row(idx+1)=1;
        row(idx+2)=-1;
        row(idx+3)=1;
        row(idx+4)=-1;
        row(idx+5)=-1;

        Aeq=[Aeq;row];
        beq=[beq; Load_tot(idx_start+i-1)-PV_tot(idx_start+i-1)];
    end

    % SOC dynamics
    for i=1:Nh-1
        row=zeros(1,nvar);
        idx=(i-1)*6; idx2=i*6;

        row(idx+6)=1;
        row(idx+3)=-dt;
        row(idx+4)= dt;
        row(idx2+6)=-1;

        Aeq=[Aeq;row];
        beq=[beq;0];
    end

    % Initial SOC
    row=zeros(1,nvar);
    row(6)=1;
    Aeq=[Aeq;row];
    beq=[beq;SOC_now];

    %% INEQUALITY (Transformer limit)
    A=[]; b=[];
    for i=1:Nh
        idx=(i-1)*6;

        r1=zeros(1,nvar); r1(idx+1)=1;  r1(idx+2)=-1;
        r2=zeros(1,nvar); r2(idx+1)=-1; r2(idx+2)=1;

        A=[A;r1;r2];
        b=[b;Pgrid_max;2*Pgrid_max/3];
    end

    %% SOLVE
    x = linprog(f,A,b,Aeq,beq,lb,ub,options);
    x = reshape(x,6,Nh)';

    %% APPLY FIRST STEP
    Pimp(k)=x(1,1);
    Pexp(k)=x(1,2);
    Curt(k)=x(1,5);

    Pbat_k = x(1,3)-x(1,4);
    Pbat(k)=Pbat_k;

    SOC_now = SOC_now + dt*(x(1,4)-x(1,3));
    SOC(k)=SOC_now;

    total_cost = total_cost + f(1:6)'*x(1,:)';
end

SOC_pct = 100*SOC/Ebat;

disp(['Total daily cost = ',num2str(total_cost)])

%% =========================================================
%% VOLTAGE CALCULATION USING SENSITIVITY METHOD
%% =========================================================

r_per_km = 0.443;

dist_m = [23.35 20.42 20.09 21.74 23.76 24.06 22 21.4 ...
          32.07 24.19 13.57 21.79 28.15 19.78];

dist_km = dist_m/1000;
R = r_per_km * dist_km;
c = cumsum(R);

n = 14;
S = zeros(n);
for i = 1:n
    for j = 1:n
        S(i,j) = c(min(i,j));
    end
end
S=S*2.5; % Adjust sensitivity factor as needed
%% Per-unit conversion
V_base = 400;
S_base = 100e3;

Z_base = V_base^2 / S_base;
S_pu = S / Z_base;

P_net = P_load - P_solar;        % kW
P_pu = P_net / (S_base/1000);

V_pu = zeros(size(P_pu));

for t = 1:size(P_pu,1)
    V_pu(t,:) = 1 - S_pu * P_pu(t,:)';
end

V_volts = V_pu * V_base;

%% =========================================================
%% VOLTAGE PLOTS (48h)
%% =========================================================
figure('Name','Pole Voltages (48h)','Position',[100 50 1200 900])

for p = 1:Npoles
    subplot(4,4,p)
    plot(time48,V_volts(:,p),'LineWidth',1.2); hold on
    yline(0.95*V_base,'--r');
    yline(1.05*V_base,'--r');
    title(['Pole ' num2str(p)])
    xlabel('Hour')
    ylabel('Voltage (V)')
    grid on
end
sgtitle('Pole-wise Voltage Profiles (Sensitivity Method)')


%% =========================================================
%% SYSTEM LEVEL PLOTS (FIRST DAY)
%% =========================================================
time = (0:Nday-1)*dt;

figure

subplot(5,1,1)
plot(time,Load_tot(1:Nday),'k', ...
     time,PV_tot(1:Nday),'g','LineWidth',1.4)
legend('Load','PV')
title('Load & PV')
grid on

subplot(5,1,2)
plot(time,Pimp,'b'); hold on
plot(time,Pexp,'r')
yline(Pgrid_max,'--k')
title('Grid Power')
grid on

subplot(5,1,3)
plot(time,Pbat); yline(0)
title('Battery Power')
grid on

subplot(5,1,4)
plot(time,SOC_pct); ylim([0 100])
title('SOC (%)')
grid on

subplot(5,1,5)
plot(time,Curt)
title('Curtailment')
grid on
xlabel('Hour')

%% =========================================================
%% EXPORT TO EXCEL (SYSTEM + VOLTAGE + ENERGY FLOWS)
%% =========================================================

SystemTable = table;

SystemTable.Time = timestamps(1:Nday);

% System power flows
SystemTable.Load_total      = Load_tot(1:Nday)';
SystemTable.Solar_total     = PV_tot(1:Nday)';
SystemTable.Curtailment     = Curt;

% Grid and battery
SystemTable.P_import        = Pimp;
SystemTable.P_export        = Pexp;
SystemTable.P_battery       = Pbat;
SystemTable.SOC_percent     = SOC_pct;

% % Pole voltages
% for p=1:14
%     SystemTable.(['V_pole_' num2str(p)]) = V_volts(1:Nday,p);
% end

writetable(SystemTable,'MPC_Results_PoleWise.xlsx');

disp('Results exported to MPC_Results_PoleWise.xlsx')

%% =========================================================
%% PERFORMANCE METRICS (for comparison)
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