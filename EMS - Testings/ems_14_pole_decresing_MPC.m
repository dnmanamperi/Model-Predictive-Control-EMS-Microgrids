% % MATLAB Linear Programming (LP) based MPC Energy Management System for a 14-pole LV feeder microgrid with:

% % • different load profiles per pole
% % • some poles with PV
% % • voltage/over-generation problems without EMS
% % • battery added
% % • 15-min resolution for 24 h (96 steps)
% % • LP optimization
% % • costs (import/export/degradation/curtailment)
% % • constraints + equations
% % • plots for loads, PV, results

% % | Type      | Poles | Description   |
% % | --------- | ----- | ------------- |
% % | Load only | 6     | houses        |
% % | PV + load | 6     | rooftop solar |
% % | Large PV  | 2     | shops/schools |

% % | Pole  | PV inverter (kW) |
% % | ----- | ---------------- |
% % | 1–6   | 0                |
% % | 7–10  | 8                |
% % | 11–12 | 12               |
% % | 13–14 | 20               |

% % ================== Optimization Problem ==================
% % Objective Function:
% % Minimize the total operational cost over horizon N
% %
% %   min  sum_{k=1}^{N} (
% %        c_imp  * P_imp(k)  ...
% %      - c_exp  * P_exp(k)  ...
% %      + c_deg  * |P_bat(k)| ...
% %      + c_curt * P_curt(k)
% %   ) * Δt
% %
% % -----------------------------------------------------------
% % Power Balance Constraint (for each time step k):
% %
% %   P_imp(k) - P_exp(k) + P_bat(k) + (PV(k) - P_curt(k)) = Load(k)
% %
% % -----------------------------------------------------------
% % Where:
% %   P_imp   : Grid import power (kW)
% %   P_exp   : Grid export power (kW)
% %   P_bat   : Battery charging(+)/discharging(-) power (kW)
% %   PV      : Available solar power (kW)
% %   P_curt  : Curtailed solar power (kW)
% %   Load    : Load demand (kW)
% %   c_imp   : Import tariff (LKR/kWh)
% %   c_exp   : Export tariff (LKR/kWh)
% %   c_deg   : Battery degradation cost (LKR/kWh)
% %   c_curt  : Curtailment penalty (LKR/kWh)
% %   Δt      : Time step duration (hours)
% % ==========================================================

% % ### Control Strategy
% % A receding-horizon MPC strategy is used: (THIS IS DECREASING MPC)
% % - At each time step, the LP is solved over the remaining horizon  //That method is only a simple “finite-day optimization”, not real MPC.
% % - Only the first control action is applied
% % - SOC is updated using measured battery state
% % - The horizon is shifted forward



% %  https://www.mdpi.com/2226-4310/12/2/111
% % https://chatgpt.com/share/6982e6be-6e70-8010-ba40-4fb42c50b308
% % https://chatgpt.com/share/6982e6be-6e70-8010-ba40-4fb42c50b308

% clc; clear; close all;

% rng(40);   % <<< fix random seed for reproducibility

% %% PARAMETERS
% Npoles = 14;
% dt = 0.25;        % hours
% N = 96;           % 24h horizon

% %% ---------- LOAD PROFILES ----------
% t = (0:N-1)/4;

% Load = zeros(Npoles,N);

% for i=1:Npoles
%     base = 2 + 0.5*rand;
%     evening = 2.5*exp(-((t-19).^2)/10);
%     morning = 1.5*exp(-((t-7).^2)/8);
%     Load(i,:) = base + morning + evening + 0.3*randn(1,N);
% end

% %% ---------- PV PROFILES ----------
% %% ---------- REALISTIC PV PROFILES ----------

% cap = [zeros(1,6) 8 8 8 8 12 12 20 20];   % inverter capacities

% PV = zeros(Npoles,N);

% % clear sky envelope (bell shape)
% env = exp(-((t-12).^2)/18);
% env = env / max(env);

% for i = 1:Npoles
    
%     if cap(i)==0
%         continue
%     end
    
%     % slow cloud variation (minutes-hours)
%     slow_noise = movmean(randn(1,N),6);
%     slow_noise = 1 + 0.35*slow_noise;
    
%     % fast cloud flicker (seconds-minutes)
%     fast_noise = 1 + 0.15*randn(1,N);
    
%     % sudden cloud drops (ramps)
%     ramp = ones(1,N);
%     events = randi([2 5]); % number of clouds
    
%     for e = 1:events
%         c = randi([20 80]);     % center time
%         w = randi([3 8]);       % duration
%         ramp(c:min(N,c+w)) = ramp(c:min(N,c+w))*rand*0.3;
%     end
    
%     pv = cap(i) * env .* slow_noise .* fast_noise .* ramp;
    
%     % inverter clipping + no negative
%     pv = max(0, min(cap(i), pv));
    
%     PV(i,:) = pv;
% end

% % PV_tot = sum(PV);


% Load_tot = sum(Load);
% PV_tot   = sum(PV);

% %% ---------- PLOTS ----------
% figure
% for i=1:Npoles
%     subplot(4,4,i)
%     plot(t,Load(i,:),'b',t,PV(i,:),'r')
%     title(['Pole ' num2str(i)])
% end
% sgtitle('Load and PV profiles')

% %% ---------- BATTERY ----------
% Ebat = 50;    % kWh
% Pbat_max = 20;
% % SOC0 = 0.5*Ebat;

% SOCmin = 0.20*Ebat;
% SOCmax = 0.80*Ebat;

% SOC0 = 0.50*Ebat;    % initial 50%
% SOCend = 0.50*Ebat;  % final 50%

% %% ---------- PRICES ----------
% c_imp = 30;
% c_exp = 20;
% c_deg = 2;
% c_curt = 1;

% %% ---------- LP VARIABLES ----------
% % order:
% % [Pimp Pexp Pbat+ Pbat- Curt SOC]

% nvar = 6*N;

% f = zeros(nvar,1);

% for k=1:N
%     idx=(k-1)*6;
%     f(idx+1)=c_imp*dt;
%     f(idx+2)=-c_exp*dt;
%     f(idx+3)=c_deg*dt;
%     f(idx+4)=c_deg*dt;
%     f(idx+5)=c_curt*dt;
% end

% %% ---------- BOUNDS ----------
% lb=zeros(nvar,1);
% ub=inf(nvar,1);

% for k=1:N
%     idx=(k-1)*6;
    
%     ub(idx+1)=250;
%     ub(idx+2)=250;
%     ub(idx+3)=Pbat_max;
%     ub(idx+4)=Pbat_max;
%     ub(idx+5)=PV_tot(k);
%     lb(idx+6)=SOCmin;
%     ub(idx+6)=SOCmax;
    
% end

% %% ---------- CONSTRAINTS ----------
% Aeq=[];
% beq=[];

% % Power balance
% for k=1:N
%     row=zeros(1,nvar);
%     idx=(k-1)*6;
    
%     row(idx+1)=1;
%     row(idx+2)=-1;
%     row(idx+3)=1;
%     row(idx+4)=-1;
%     row(idx+5)=-1;
    
%     Aeq=[Aeq;row];
%     beq=[beq; Load_tot(k)-PV_tot(k)];
% end

% % SOC dynamics
% for k=1:N-1
%     row=zeros(1,nvar);
    
%     idx=(k-1)*6;
%     idx2=k*6;
    
%     row(idx+6)=1;
%     row(idx+3)=-dt;
%     row(idx+4)=+dt;
    
%     row(idx2+6)=-1;
    
%     Aeq=[Aeq;row];
%     beq=[beq;0];
% end

% % initial SOC
% row=zeros(1,nvar);
% row(6)=1;
% Aeq=[Aeq;row];
% beq=[beq;SOC0];

% % final SOC constraint
% row=zeros(1,nvar);
% row((N-1)*6 + 6) = 1;   % SOC at last step
% Aeq=[Aeq;row];
% beq=[beq;SOCend];


% %% ---------- TRANSFORMER / GRID LIMITS ----------

% Pgrid_max = 50;   % available capacity for this feeder only (kW)

% % A = [];
% % b = [];

% % for k = 1:N

% %     row1 = zeros(1,nvar);   % import limit
% %     row2 = zeros(1,nvar);   % export limit

% %     idx=(k-1)*6;

% %     % Pimp - Pexp <= Pgrid_max
% %     row1(idx+1)=1;
% %     row1(idx+2)=-1;

% %     % -(Pimp - Pexp) <= Pgrid_max
% %     row2(idx+1)=-1;
% %     row2(idx+2)=1;

% %     A=[A; row1; row2];
% %     b=[b; Pgrid_max; Pgrid_max/3];
% % end



% %% ---------- SOLVE ----------
% %% ==========================================================
% %% MPC EMS (rolling horizon linear programming)
% %% ==========================================================

% Pimp  = zeros(N,1);
% Pexp  = zeros(N,1);
% Pbat  = zeros(N,1);
% Curt  = zeros(N,1);
% SOC   = zeros(N,1);

% SOC_now = SOC0;
% total_cost = 0;

% options = optimoptions('linprog','Display','none');

% for k = 1:N
    
%     Nh = N-k+1;           % remaining horizon
%     nvar = 6*Nh;
    
%     %% ---------- COST ----------
%     f = zeros(nvar,1);
%     for i=1:Nh
%         idx=(i-1)*6;
%         f(idx+1)=c_imp*dt;
%         f(idx+2)=-c_exp*dt;
%         f(idx+3)=c_deg*dt;
%         f(idx+4)=c_deg*dt;
%         f(idx+5)=c_curt*dt;
%     end
    
%     %% ---------- BOUNDS ----------
%     lb=zeros(nvar,1);
%     ub=inf(nvar,1);
    
%     for i=1:Nh
%         idx=(i-1)*6;
%         ub(idx+1)=250;
%         ub(idx+2)=250;
%         ub(idx+3)=Pbat_max;
%         ub(idx+4)=Pbat_max;
%         ub(idx+5)=PV_tot(k+i-1);
        
%         lb(idx+6)=SOCmin;
%         ub(idx+6)=SOCmax;
%     end
    
%     %% ---------- CONSTRAINTS ----------
%     Aeq=[]; beq=[];
    
%     % Power balance
%     for i=1:Nh
%         row=zeros(1,nvar);
%         idx=(i-1)*6;
        
%         row(idx+1)=1;
%         row(idx+2)=-1;
%         row(idx+3)=1;
%         row(idx+4)=-1;
%         row(idx+5)=-1;
        
%         Aeq=[Aeq;row];
%         beq=[beq; Load_tot(k+i-1)-PV_tot(k+i-1)];
%     end
    
%     % SOC dynamics
%     for i=1:Nh-1
%         row=zeros(1,nvar);
%         idx=(i-1)*6;
%         idx2=i*6;
        
%         row(idx+6)=1;
%         row(idx+3)=-dt;
%         row(idx+4)=+dt;
%         row(idx2+6)=-1;
        
%         Aeq=[Aeq;row];
%         beq=[beq;0];
%     end
    
%     % initial SOC (measured)
%     row=zeros(1,nvar);
%     row(6)=1;
%     Aeq=[Aeq;row];
%     beq=[beq;SOC_now];
    
%     A = [];
%     b = [];
    
%     % ---------- terminal SOC ----------
%     row=zeros(1,nvar);
%     row((Nh-1)*6 + 6)=1;
%     Aeq=[Aeq;row];
%     beq=[beq;SOCend];
    
%     for i = 1:Nh
        
%         row1 = zeros(1,nvar);
%         row2 = zeros(1,nvar);
        
%         idx=(i-1)*6;
        
%         %  Pimp - Pexp <= +limit
%         row1(idx+1)=1;
%         row1(idx+2)=-1;
        
%         % -(Pimp - Pexp) <= export_limit
%         row2(idx+1)=-1;
%         row2(idx+2)=1;
        
%         A=[A;row1;row2];
%         b=[b; Pgrid_max; Pgrid_max/3];
%     end
    
    
%     %% ---------- SOLVE LP ----------
%     x = linprog(f,A,b,Aeq,beq,lb,ub,options);
%     x = reshape(x,6,Nh)';
    
%     %% ---------- APPLY FIRST STEP ONLY ----------
%     Pimp(k)=x(1,1);
%     Pexp(k)=x(1,2);
%     Pbat(k)=x(1,3)-x(1,4);
%     Curt(k)=x(1,5);
%     % SOC(k)=x(1,6);
    
%     % SOC_now = SOC(k);    % update for next step
%     Pbat_k = x(1,3) - x(1,4);
    
%     SOC_now = SOC_now + dt*(x(1,4) - x(1,3));   % charge minus discharge
    
%     SOC(k) = SOC_now;
%     Pbat(k) = Pbat_k;
%     total_cost = total_cost + f(1:6)'*x(1,:)';
% end

% SOC_pct = 100*SOC/Ebat;

% disp(['Total daily cost (MPC) = ' num2str(total_cost)])



% %% ---------- EXTRACT ----------
% % x=reshape(x,6,N)';

% % Pimp=x(:,1);
% % Pexp=x(:,2);
% % Pbat=x(:,3)-x(:,4);
% % Curt=x(:,5);
% % SOC=x(:,6);


% % SOC_pct = 100 * SOC / Ebat;
% %% ---------- RESULTS ----------
% figure
% plot(t,Load_tot,'k',t,PV_tot,'g')
% legend('Load','PV')
% title('Total Load & PV')

% figure
% plot(t,Pimp,'b',t,Pexp,'r')
% legend('Import','Export')
% title('Grid Power')

% figure
% plot(t,Pbat)
% title('Battery Power')

% figure
% plot(t,SOC)
% title('Battery SOC')

% figure
% plot(t,Curt)
% title('Curtailment')

% cost=f'*x(:);
% disp(['Total daily cost = ' num2str(cost)])



% %% ---------- COMBINED EMS SUMMARY FIGURE ----------

% figure('Name','EMS Operation Summary','Position',[100 50 900 900])

% % -------- 1 Load & PV --------
% subplot(5,1,1)
% plot(t,Load_tot,'k','LineWidth',1.4); hold on
% plot(t,PV_tot,'g','LineWidth',1.4)
% ylabel('kW')
% title('Load vs PV')
% legend('Load','PV')
% grid on

% % -------- 2 Grid exchange --------
% subplot(5,1,2)
% plot(t,Pimp,'b','LineWidth',1.3); hold on
% plot(t,Pexp,'r','LineWidth',1.3)
% yline(Pgrid_max,'--k');
% yline(-Pgrid_max,'--k');
% ylabel('kW')
% title('Grid Import / Export (Transformer limits shown)')
% legend('Import','Export')
% grid on

% % -------- 3 Battery power --------
% subplot(5,1,3)
% plot(t,Pbat,'LineWidth',1.4)
% yline(0,'k')
% ylabel('kW')
% title('Battery Power (+ discharge, – charge)')
% grid on

% % -------- 4 SOC --------
% subplot(5,1,4)
% plot(t,SOC,'LineWidth',1.4)
% ylabel('kWh')
% title('Battery SOC')
% grid on

% % -------- 5 Curtailment --------
% subplot(5,1,5)
% plot(t,Curt,'LineWidth',1.4)
% xlabel('Time (hour)')
% ylabel('kW')
% title('PV Curtailment')
% grid on


% figure
% plot(t,SOC_pct,'LineWidth',1.4)
% ylim([0 100])
% ylabel('%')
% title('Battery SOC (%)')
% grid on















%clean code

%% =========================================================
%  Linear Programming MPC – Energy Management System
%  14-pole LV feeder with PV + Battery + Transformer limits
%  15-min resolution, 24h horizon
%% =========================================================
clc; clear; close all;
rng(40);

%% =========================================================
%% SYSTEM SETTINGS
%% =========================================================
Npoles = 14;
dt  = 0.25;          % hour
N   = 96;            % 24 h

t = (0:N-1)*dt;

%% =========================================================
%% LOAD PROFILES (random realistic houses)
%% =========================================================
Load = zeros(Npoles,N);

for i=1:Npoles
    base = 2 + 0.5*rand;
    morning = 1.5*exp(-((t-7).^2)/8);
    evening = 2.5*exp(-((t-19).^2)/10);
    Load(i,:) = base + morning + evening + 0.3*randn(1,N);
end

%% =========================================================
%% PV PROFILES (realistic noisy + cloud drops)
%% =========================================================
cap = [zeros(1,6) 8 8 8 8 12 12 20 20];
PV = zeros(Npoles,N);

env = exp(-((t-12).^2)/18);
env = env/max(env);

for i=1:Npoles
    if cap(i)==0, continue; end

    slow = 1 + 0.35*movmean(randn(1,N),6);
    fast = 1 + 0.15*randn(1,N);

    ramp = ones(1,N);
    for e=1:randi([2 5])
        c=randi([20 80]); w=randi([3 8]);
        ramp(c:min(N,c+w)) = 0.3*rand;
    end

    pv = cap(i)*env.*slow.*fast.*ramp;
    PV(i,:) = max(0,min(cap(i),pv));
end

Load_tot = sum(Load);
PV_tot   = sum(PV);

%% =========================================================
%% BATTERY
%% =========================================================
Ebat = 50;           % kWh
Pbat_max = 20;       % kW

SOC0  = 0.5*Ebat;
SOCend = 0.5*Ebat;

SOCmin = 0.2*Ebat;
SOCmax = 0.8*Ebat;

%% =========================================================
%% GRID
%% =========================================================
Pgrid_max = 50;      % kW transformer limit

%% =========================================================
%% COSTS (LKR/kWh)
%% =========================================================
c_imp  = 30;
c_exp  = 20;
c_deg  = 2;
c_curt = 1;

%% =========================================================
%% STORAGE FOR RESULTS
%% =========================================================
Pimp=zeros(N,1);
Pexp=zeros(N,1);
Pbat=zeros(N,1);
Curt=zeros(N,1);
SOC =zeros(N,1);

SOC_now = SOC0;
total_cost = 0;

options = optimoptions('linprog','Display','none');

%% =========================================================
%% MPC LOOP
%% =========================================================
for k=1:N
    
    Nh = N-k+1;
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
        ub(idx+5)=PV_tot(k+i-1);
        lb(idx+6)=SOCmin;
        ub(idx+6)=SOCmax;
    end

    %% =====================================================
    %% EQUALITY CONSTRAINTS
    %% =====================================================
    Aeq=[]; beq=[];

    % ----- Power balance -----
    for i=1:Nh
        row=zeros(1,nvar);
        idx=(i-1)*6;

        row(idx+1)=1;
        row(idx+2)=-1;
        row(idx+3)=1;
        row(idx+4)=-1;
        row(idx+5)=-1;

        Aeq=[Aeq;row];
        beq=[beq;Load_tot(k+i-1)-PV_tot(k+i-1)];
    end

    % ----- SOC dynamics -----
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

    % initial SOC
    row=zeros(1,nvar); row(6)=1;
    Aeq=[Aeq;row]; beq=[beq;SOC_now];

    % terminal SOC
    row=zeros(1,nvar); row((Nh-1)*6+6)=1;
    Aeq=[Aeq;row]; beq=[beq;SOCend];

    %% =====================================================
    %% INEQUALITY CONSTRAINTS (Transformer limits)
    %% =====================================================
    A=[]; b=[];

    for i=1:Nh
        idx=(i-1)*6;

        r1=zeros(1,nvar); r1(idx+1)=1;  r1(idx+2)=-1;
        r2=zeros(1,nvar); r2(idx+1)=-1; r2(idx+2)=1;

        A=[A;r1;r2];
        b=[b;Pgrid_max;Pgrid_max/3];
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

disp(['Total daily cost = ',num2str(total_cost)])

%% =========================================================
%% PLOTS
%% =========================================================
figure

subplot(5,1,1)
plot(t,Load_tot,'k',t,PV_tot,'g')
title('Load & PV'); grid on

subplot(5,1,2)
plot(t,Pimp,'b',t,Pexp,'r')
yline(Pgrid_max,'--k')
title('Grid Power'); grid on

subplot(5,1,3)
plot(t,Pbat)
yline(0,'k')
title('Battery Power'); grid on

subplot(5,1,4)
plot(t,SOC_pct)
ylim([0 100])
title('SOC (%)'); grid on

subplot(5,1,5)
plot(t,Curt)
title('Curtailment'); grid on
xlabel('Hour')


%% =========================================================
%% SINGLE FIGURE – 14 SUBPLOTS (Load + PV per pole)
%% =========================================================

figure('Name','Per-Pole Load & PV Profiles','Position',[100 80 1200 800])

rows = 4;
cols = 4;

for i = 1:Npoles
    
    subplot(rows,cols,i)
    
    plot(t,Load(i,:),'b','LineWidth',1.2); hold on
    plot(t,PV(i,:),'r','LineWidth',1.2)
    
    title(['Pole ' num2str(i) '  (PV cap = ' num2str(cap(i)) ' kW)'])
    xlabel('Hour')
    ylabel('kW')
    grid on
    
    % auto same scale for better comparison
    ylim([0 max([Load(:);PV(:)])*1.05])
end

legend('Load','PV')
sgtitle('Load and Solar Profiles for All 14 Poles')