%% =========================================================
% TRUE CONSTANT-HORIZON MPC ENERGY MANAGEMENT SYSTEM
% 48h simulation
% 24h rolling optimization horizon
%% =========================================================
clc; clear; close all;
rng(40);

%% =========================================================
%% BASIC SETTINGS
%% =========================================================
dt  = 0.25;               % 15 min
Nday = 96;               % 24h horizon
Nsim = 192;              % 48h data (needed for rolling)

Npoles = 14;
t = (0:Nsim-1)*dt;

%% =========================================================
%% LOAD GENERATION (48h)
%% =========================================================
Load = zeros(Npoles,Nsim);

for i=1:Npoles
    base = 2 + 0.5*rand;

    morning = 1.5*exp(-((mod(t,24)-7).^2)/8);
    evening = 2.5*exp(-((mod(t,24)-19).^2)/10);

    Load(i,:) = base + morning + evening + 0.3*randn(1,Nsim);
end

%% =========================================================
%% PV GENERATION (48h)
%% =========================================================
cap = [zeros(1,6) 8 8 8 8 12 12 20 20];
PV = zeros(Npoles,Nsim);

env = exp(-((mod(t,24)-12).^2)/18);
env = env/max(env);

for i=1:Npoles
    if cap(i)==0, continue; end

    slow = 1 + 0.35*movmean(randn(1,Nsim),6);
    fast = 1 + 0.15*randn(1,Nsim);

    pv = cap(i)*env.*slow.*fast;
    PV(i,:) = max(0,min(cap(i),pv));
end

Load_tot = sum(Load);
PV_tot   = sum(PV);


%% =========================================================
%% POLE-WISE LOAD & PV PROFILES (14 subplots in one figure)
%% =========================================================

time = (0:length(Load(1,:))-1)*dt;

figure('Name','Pole-wise Load & PV Profiles','Position',[100 50 1200 900])

for p = 1:Npoles
    
    subplot(4,4,p)   % 4x4 grid
    
    plot(time, Load(p,:), 'b','LineWidth',1.2); hold on
    plot(time, PV(p,:),   'r','LineWidth',1.2);
    
    title(['Pole ' num2str(p)])
    xlabel('Hour')
    ylabel('kW')
    grid on
    
    if p==1
        legend('Load','PV')   % only once (clean look)
    end
end

sgtitle('Individual Pole Load and Solar Profiles')



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
c_exp  = 20;
c_deg  = 2;
c_curt = 1;

%% =========================================================
%% RESULT STORAGE (only first day plotted)
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
    
    Nh = Nday;               % <<< CONSTANT horizon
    idx_start = k;
    idx_end   = k+Nh-1;
    
    nvar = 6*Nh;

    %% ---------- COST ----------
    f=zeros(nvar,1);
    for i=1:Nh
        idx=(i-1)*6;
        f(idx+1)=c_imp*dt;
        f(idx+2)=-c_exp*dt;
        f(idx+3)=c_deg*dt;
        f(idx+4)=c_deg*dt;
        f(idx+5)=c_curt*dt;
    end

    %% ---------- BOUNDS ----------
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

    %% =====================================================
    %% EQUALITY CONSTRAINTS
    %% =====================================================
    Aeq=[]; beq=[];

    % power balance
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

    % measured initial SOC
    row=zeros(1,nvar);
    row(6)=1;
    Aeq=[Aeq;row];
    beq=[beq;SOC_now];

    %% =====================================================
    %% INEQUALITY (Transformer)
    %% =====================================================
    A=[]; b=[];
    for i=1:Nh
        idx=(i-1)*6;

        r1=zeros(1,nvar); r1(idx+1)=1;  r1(idx+2)=-1;
        r2=zeros(1,nvar); r2(idx+1)=-1; r2(idx+2)=1;

        A=[A;r1;r2];
        b=[b;Pgrid_max;2*Pgrid_max/3];
    end

    %% ---------- SOLVE ----------
    x = linprog(f,A,b,Aeq,beq,lb,ub,options);
    x = reshape(x,6,Nh)';

    %% ---------- APPLY FIRST STEP ----------
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

disp(['Total daily cost (TRUE MPC) = ',num2str(total_cost)])

%% =========================================================
%% PLOTS
%% =========================================================
figure
% subplot(5,1,1)
% plot((0:Nday-1)*dt,Load_tot(1:Nday),'k',PV_tot(1:Nday),'g')
% title('Load & PV'); grid on

% time = (0:Nday-1)*dt;

% subplot(5,1,1)
% plot(time,Load_tot(1:Nday),'k', ...
%      time,PV_tot(1:Nday),'g','LineWidth',1.4)
% legend('Load','PV')
% title('Load & PV')
% grid on

subplot(5,1,1)
plot(Load_tot(1:Nday),'k','LineWidth',1.4); hold on
plot(PV_tot(1:Nday),'g','LineWidth',1.4)
legend('Load','PV')
title('Load & PV')
grid on
xlim([1 Nday])


subplot(5,1,2)
plot(Pimp,'b'); hold on; plot(Pexp,'r')
yline(Pgrid_max,'--k')
xlim([1 Nday])
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
