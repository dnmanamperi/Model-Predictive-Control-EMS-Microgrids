%% =========================================================
% TWO-LAYER CONSTANT-HORIZON MPC EMS
% USING:
% 1) Forecasted Load & PV from Excel
% 2) Voltage from Sensitivity Matrix
%% =========================================================
clc; clear; close all;

%% =========================================================
%% BASIC SETTINGS
%% =========================================================
dt  = 0.25;                 % 15 min
Nday = 96;                  % 24h MPC horizon
Nsim = 192;                 % 48h data length
Npoles = 14;

%% =========================================================
%% READ FORECAST DATA FROM EXCEL
%% =========================================================
data = readtable('afterforecasting.xlsx');

P_load  = data{:,2:15};      % kW (14 poles)
P_solar = data{:,16:29};     % kW

Load = P_load';
PV   = P_solar';

Load_tot = sum(Load);
PV_tot   = sum(PV);

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
%% BATTERY PARAMETERS
%% =========================================================
Ebat = 50;
Pbat_max = 20;
SOCmin = 0.2*Ebat;
SOCmax = 0.8*Ebat;
SOC0   = 0.5*Ebat;

%% =========================================================
%% GRID + COST PARAMETERS
%% =========================================================
Pgrid_max = 50;
c_deg  = 2;
c_curt = 1;

%% =========================================================
%% UPPER LAYER SETTINGS
%% =========================================================
V_ref = 1.0;
k_voltage = 50;

Texp_base = 20;
Timp_base = 30;
delta = 0.2;

%% =========================================================
%% STORAGE VARIABLES
%% =========================================================
Pimp=zeros(Nday,1);
Pexp=zeros(Nday,1);
Pbat=zeros(Nday,1);
Curt=zeros(Nday,1);
SOC =zeros(Nday,1);

Pexp_pole = zeros(Npoles,Nday);
Curt_pole = zeros(Npoles,Nday);
Texp_pole = zeros(Npoles,Nday);

SOC_now = SOC0;
total_cost = 0;

options = optimoptions('linprog','Display','none');
Timp_prev = Timp_base;

%% =========================================================
%% TRUE CONSTANT-HORIZON MPC
%% =========================================================
for k=1:Nday
    
    Nh = Nday;
    idx_start = k;
    
    %% =============================
    %% UPPER LAYER LP
    %% =============================
    w1 = 1; w2 = 3; w3 = 0.2;
    
    f_up = [0; w3; w1; w2];
    
    lb_up = [(1-delta)*Timp_base; 0; 0; 0];
    ub_up = [(1+delta)*Timp_base; 5; inf; inf];
    
    A_up = [
        1  0  -1   0;
       -1  0  -1   0;
        1  0   0  -1;
       -1  0   0  -1];
    
    b_up = [
        Timp_base;
       -Timp_base;
        Timp_prev;
       -Timp_prev];
    
    x_up = linprog(f_up,A_up,b_up,[],[],lb_up,ub_up,options);
    
    T_imp = x_up(1);
    phi   = x_up(2);
    Timp_prev = T_imp;
    
    %% Voltage-based export tariff
    Texp_voltage = Texp_base + ...
        k_voltage*(V_ref - V_poles(:,idx_start));
    
    T_exp = Texp_voltage + phi;
    
    %% =============================
    %% LOWER LAYER MPC
    %% =============================
    nv_per = 1 + Npoles + 2 + Npoles + 1;
    nvar = Nh*nv_per;
    
    f_mpc=zeros(nvar,1);
    
    for i=1:Nh
        idx=(i-1)*nv_per;
        f_mpc(idx+1)=T_imp*dt;
        f_mpc(idx+2:idx+1+Npoles)=-dt*T_exp;
        f_mpc(idx+2+Npoles)=c_deg*dt;
        f_mpc(idx+3+Npoles)=c_deg*dt;
        f_mpc(idx+4+Npoles:idx+3+2*Npoles)=c_curt*dt;
    end
    
    lb=zeros(nvar,1); ub=inf(nvar,1);
    
    for i=1:Nh
        idx=(i-1)*nv_per;
        
        ub(idx+1)=Pgrid_max;
        ub(idx+2:idx+1+Npoles)=PV(:,idx_start+i-1);
        ub(idx+2+Npoles)=Pbat_max;
        ub(idx+3+Npoles)=Pbat_max;
        ub(idx+4+Npoles:idx+3+2*Npoles)=PV(:,idx_start+i-1);
        
        lb(idx+4+2*Npoles)=SOCmin;
        ub(idx+4+2*Npoles)=SOCmax;
    end
    
    %% Equality constraints
    Aeq=[]; beq=[];
    
    % Power balance
    for i=1:Nh
        row=zeros(1,nvar);
        idx=(i-1)*nv_per;
        
        row(idx+1)=1;
        row(idx+2:idx+1+Npoles)=-1;
        row(idx+2+Npoles)=1;
        row(idx+3+Npoles)=-1;
        row(idx+4+Npoles:idx+3+2*Npoles)=-1;
        
        beq_i = sum(Load(:,idx_start+i-1)) ...
              - sum(PV(:,idx_start+i-1));
        
        Aeq=[Aeq;row];
        beq=[beq;beq_i];
    end
    
    % SOC dynamics
    for i=1:Nh-1
        row=zeros(1,nvar);
        idx=(i-1)*nv_per;
        idx2=i*nv_per;
        
        row(idx+4+2*Npoles)=1;
        row(idx+2+Npoles)=-dt;
        row(idx+3+Npoles)=dt;
        row(idx2+4+2*Npoles)=-1;
        
        Aeq=[Aeq;row];
        beq=[beq;0];
    end
    
    % Initial SOC
    row=zeros(1,nvar);
    row(4+2*Npoles)=1;
    Aeq=[Aeq;row];
    beq=[beq;SOC_now];
    
    %% Solve
    x = linprog(f_mpc,[],[],Aeq,beq,lb,ub,options);
    x = reshape(x,nv_per,Nh)';
    
    %% Apply first control
    Pimp(k)=x(1,1);
    Pexp_pole(:,k)=x(1,2:1+Npoles)';
    Pexp(k)=sum(Pexp_pole(:,k));
    
    Curt_pole(:,k)=x(1,4+Npoles:3+2*Npoles)';
    Curt(k)=sum(Curt_pole(:,k));
    
    Pbat_k = x(1,3+Npoles)-x(1,2+Npoles);
    Pbat(k)=Pbat_k;
    
    SOC_now = SOC_now + dt*(x(1,3+Npoles)-x(1,2+Npoles));
    SOC(k)=SOC_now;
    
    Texp_pole(:,k)=T_exp;
    
    total_cost = total_cost + f_mpc(1:nv_per)'*x(1,:)';
end

SOC_pct = 100*SOC/Ebat;
disp(['Total Cost = ',num2str(total_cost),' LKR'])

%% =========================================================
%% SYSTEM OVERVIEW PLOTS
%% =========================================================
time_plot = (0:Nday-1)*dt;

figure('Name','System Overview','Position',[100 100 1000 800])

subplot(5,1,1)
plot(time_plot, Load_tot(1:Nday),'k','LineWidth',1.4); hold on
plot(time_plot, PV_tot(1:Nday),'g','LineWidth',1.4)
legend('Total Load','Total PV')
title('Total Load & PV')
grid on

subplot(5,1,2)
plot(time_plot,Pimp,'b','LineWidth',1.3); hold on
plot(time_plot,Pexp,'r','LineWidth',1.3)
yline(Pgrid_max,'--k')
legend('Import','Export')
title('Grid Power')
grid on

subplot(5,1,3)
plot(time_plot,Pbat,'LineWidth',1.4)
yline(0,'--k')
title('Battery Power (+ discharge)')
grid on

subplot(5,1,4)
plot(time_plot,100*SOC/Ebat,'LineWidth',1.4)
ylim([0 100])
title('Battery SOC (%)')
grid on

subplot(5,1,5)
plot(time_plot,Curt,'LineWidth',1.4)
title('Total Curtailment')
xlabel('Hour')
grid on


%% =========================================================
%% POLE-WISE VOLTAGE PROFILES
%% =========================================================
figure('Name','Pole Voltages','Position',[100 50 1200 900])

for p = 1:Npoles
    subplot(4,4,p)
    plot(time_plot, V_poles(p,1:Nday),'LineWidth',1.3)
    yline(1,'--k')
    title(['Pole ' num2str(p)])
    ylim([0.94 1.05])
    grid on
end

sgtitle('Voltage Profiles (Sensitivity Method)')

%% =========================================================
%% VOLTAGE vs EXPORT TARIFF (Pole 1 Example)
%% =========================================================
figure('Name','Voltage vs Tariff')

yyaxis left
plot(time_plot,V_poles(1,1:Nday),'LineWidth',1.4)
ylabel('Voltage (p.u.)')

yyaxis right
plot(time_plot,Texp_pole(1,:),'LineWidth',1.4)
ylabel('Export Tariff (LKR/kWh)')

xlabel('Hour')
title('Voltage–Tariff Interaction (Pole 1)')
grid on

%% =========================================================
%% POLE-WISE CURTAILMENT
%% =========================================================
figure('Name','Pole Curtailment','Position',[100 50 1200 900])

for p = 1:Npoles
    subplot(4,4,p)
    plot(time_plot,Curt_pole(p,1:Nday),'r','LineWidth',1.2)
    title(['Pole ' num2str(p)])
    grid on
end

sgtitle('Pole-wise Curtailment')


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