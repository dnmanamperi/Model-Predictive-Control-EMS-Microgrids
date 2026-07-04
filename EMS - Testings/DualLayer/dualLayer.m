%% ============================================================
%  TWO LAYER EMS + MPC (FULL IMPLEMENTATION)
%  Upper layer : tariff shaping
%  Lower layer : MPC dispatch (linprog)
% =============================================================

clear; clc; close all;

%% =============================================================
% BASIC SETTINGS
% =============================================================
dt = 0.25;                 % 15 min
H  = 24/dt;                % MPC horizon = 96
Nsim = 2*H;                % 48 h data

Npoles = 14;

t = (0:Nsim-1)*dt;

rng(1)

%% =============================================================
% LOAD PROFILE (your model kept)
% =============================================================
Load = zeros(Npoles,Nsim);

for i=1:Npoles
    base = 2 + 0.5*rand;

    morning = 1.5*exp(-((mod(t,24)-7).^2)/8);
    evening = 2.5*exp(-((mod(t,24)-19).^2)/10);

    Load(i,:) = base + morning + evening + 0.3*randn(1,Nsim);
end

Load = max(0,Load);

%% =============================================================
% SOLAR PROFILE (your model kept)
% =============================================================
PV = zeros(Npoles,Nsim);

cap = 6 + 2*rand(Npoles,1);

env = max(0, sin(pi*(mod(t,24)-6)/12));   % day-night shape

for i=1:Npoles
    slow = 1 + 0.35*movmean(randn(1,Nsim),6);
    fast = 1 + 0.15*randn(1,Nsim);

    pv = cap(i)*env.*slow.*fast;
    PV(i,:) = max(0,min(cap(i),pv));
end

%% =============================================================
% VOLTAGE MODEL (simple sensitivity approximation)
% =============================================================
Vref = 1;

sens = 0.015;   % voltage rise per kW export

V = zeros(Npoles,Nsim);

for h=1:Nsim
    net = PV(:,h) - Load(:,h);
    V(:,h) = Vref + sens*net;
end

%% =============================================================
% =============== UPPER LAYER : TARIFF GENERATOR =================
% =============================================================

T_CEB = 30 + 5*(mod(t,24)>=18 & mod(t,24)<=22);  % TOU price

delta = 0.3;

T_imp = zeros(1,Nsim);
phi   = zeros(1,Nsim);

for h=1:Nsim
    T_imp(h) = min(max(T_CEB(h), (1-delta)*T_CEB(h)), (1+delta)*T_CEB(h));
end

% battery recovery adder
phi(:) = 2;   % simple constant (can optimize later)

T_imp = T_imp + phi;

% ---------- voltage based export price -----------
Texp_base = 0.7*T_imp;
k = 80;

T_exp = zeros(Npoles,Nsim);

for h=1:Nsim
    for i=1:Npoles
        T_exp(i,h) = Texp_base(h) + k*(Vref - V(i,h));
    end
end

% curtailment penalty
c_curt = 200*ones(Npoles,Nsim);

%% =============================================================
% =============== BATTERY PARAMETERS =============================
% =============================================================
Ebat = 50;        % kWh
Pbat_max = 20;    % kW
SOCmin = 0.1;
SOCmax = 0.9;
SOC0   = 0.5;

eta_ch  = 0.95;
eta_dis = 0.95;

c_deg = 1.5;     % degradation cost

Pgrid_max = 50;

%% =============================================================
% STORAGE FOR RESULTS
% =============================================================
SOC = zeros(1,Nsim);  SOC(1)=SOC0;

Pimp_hist = zeros(1,Nsim);
Pch_hist  = zeros(1,Nsim);
Pdis_hist = zeros(1,Nsim);

Curt_hist = zeros(Npoles,Nsim);
Pexp_hist = zeros(Npoles,Nsim);

%% =============================================================
% =============== LOWER LAYER : MPC LOOP ========================
% =============================================================

opts = optimoptions('linprog','Display','none');

for kstep = 1:H

    idx = kstep:(kstep+H-1);

    Lf = Load(:,idx);
    PVf = PV(:,idx);

    Tim = T_imp(idx);
    Tex = T_exp(:,idx);
    Ccurt = c_curt(:,idx);

    %% ---------------- Decision variables count ---------------
    % [Pimp H | Pch H | Pdis H | SOC H |
    %  Curt Np*H | Pexp Np*H]

    nP = H;
    nSOC = H;
    nCurt = Npoles*H;
    nExp  = Npoles*H;

    n = nP + nP + nP + nSOC + nCurt + nExp;

    f = zeros(n,1);

    %% ---------------- Cost vector ----------------------------
    p=0;

    f(p+(1:H)) = Tim*dt;   p=p+H;
    f(p+(1:H)) = c_deg*dt; p=p+H;
    f(p+(1:H)) = c_deg*dt; p=p+H;
    p=p+H;

    f(p+(1:nCurt)) = Ccurt(:)*dt; p=p+nCurt;
    f(p+(1:nExp))  = -Tex(:)*dt;

    %% ---------------- Bounds ---------------------------------
    lb=zeros(n,1); ub=inf(n,1);

    p=0;
    ub(p+(1:H))=Pgrid_max; p=p+H;
    ub(p+(1:H))=Pbat_max;  p=p+H;
    ub(p+(1:H))=Pbat_max;  p=p+H;

    lb(p+(1:H))=SOCmin; ub(p+(1:H))=SOCmax; p=p+H;

    ub(p+(1:nCurt))=PVf(:); p=p+nCurt;
    ub(p+(1:nExp)) = Pgrid_max;

    %% ---------------- Equality constraints -------------------
    Aeq=[];
    beq=[];

    % ----- power balance
    for h=1:H
        row=zeros(1,n);

        base=(h-1)*Npoles;

        row(h)=1;

        row(H+h)=1;
        row(2*H+h)=-1;

        for i=1:Npoles
            row(4*H+base+i)=-1;
            row(4*H+nCurt+base+i)=-1;
        end

        rhs=sum(Lf(:,h)-PVf(:,h));

        Aeq=[Aeq;row];
        beq=[beq;rhs];
    end

    % ----- SOC dynamics
    for h=1:H
        row=zeros(1,n);

        soc_idx = 3*H + h;

        if h==1
            row(soc_idx)=1;
            row(H+1)= -dt*eta_ch/Ebat;
            row(2*H+1)= dt/(eta_dis*Ebat);

            Aeq=[Aeq;row];
            beq=[beq;SOC(kstep)];
        else
            row(soc_idx)=1;
            row(soc_idx-1)=-1;

            row(H+h)= -dt*eta_ch/Ebat;
            row(2*H+h)= dt/(eta_dis*Ebat);

            Aeq=[Aeq;row];
            beq=[beq;0];
        end
    end

    %% ---------------- Solve LP -------------------------------
    x = linprog(f,[],[],Aeq,beq,lb,ub,opts);

    %% ---------------- Apply first action ---------------------
    Pimp_hist(kstep)=x(1);
    Pch_hist(kstep)=x(H+1);
    Pdis_hist(kstep)=x(2*H+1);
    SOC(kstep+1)=x(3*H+1);

    Curt_hist(:,kstep)=x(4*H+(1:Npoles));
    Pexp_hist(:,kstep)=x(4*H+nCurt+(1:Npoles));
end



%% ================= COST CALCULATION (FIXED) ===================

Tim = T_imp(1:H)';                 % column

Energy_import = Pimp_hist(1:H)' * dt;
Energy_export = sum(Pexp_hist(:,1:H),1)' * dt;
Energy_deg    = (Pch_hist(1:H)' + Pdis_hist(1:H)') * dt;
Energy_curt   = sum(Curt_hist(:,1:H),1)' * dt;

cost_import = sum(Tim .* Energy_import);
revenue_exp = sum(sum(T_exp(:,1:H) .* (Pexp_hist(:,1:H)*dt)));
cost_deg    = sum(c_deg * Energy_deg);
cost_curt   = sum(sum(c_curt(:,1:H) .* (Curt_hist(:,1:H)*dt)));

Total_cost = cost_import - revenue_exp + cost_deg + cost_curt;








%% =============================================================
% ===================== PLOTS =================================
% =============================================================

figure
plot(t(1:H),sum(Load(:,1:H)),'k','LineWidth',2); hold on
plot(t(1:H),sum(PV(:,1:H)),'y','LineWidth',2)
title('Total Load vs Solar')
legend('Load','PV')

figure
plot(t(1:H),SOC(1:H),'LineWidth',2)
title('Battery SOC')

figure
plot(t(1:H),Pimp_hist(1:H),'LineWidth',2)
title('Grid Import')

figure
plot(t(1:H),Pch_hist(1:H),'LineWidth',2); hold on
plot(t(1:H),Pdis_hist(1:H),'LineWidth',2)
legend('Charge','Discharge')
title('Battery Power')

figure
plot(t(1:H),sum(Pexp_hist(:,1:H),1),'LineWidth',2)
title('Total Export')

figure
imagesc(Curt_hist(:,1:H))
colorbar
title('Pole-wise Curtailment (kW)')
xlabel('Time step')
ylabel('Pole')

figure
imagesc(V(:,1:H))
colorbar
title('Pole Voltages')
xlabel('Time step')
ylabel('Pole')




%% =============================================================
% ============== BASELINE (NO BATTERY) ==========================
% =============================================================

net = sum(Load(:,1:H) - PV(:,1:H),1);

imp0 = max(0, net);
exp0 = max(0,-net);

cost_base = sum(T_imp(1:H).*imp0*dt) - sum(mean(T_exp(:,1:H),1).*exp0*dt);

fprintf('\nBaseline cost (no battery) = %.2f Rs\n',cost_base);
fprintf('Savings using MPC         = %.2f Rs\n',cost_base-Total_cost);





%% =============================================================
% ====================== MAIN FIGURE ============================
% =============================================================

figure('Position',[100 50 1200 900])

% ---- Load vs PV
subplot(4,2,1)
plot(t(1:H),sum(Load(:,1:H)),'k','LineWidth',1.8); hold on
plot(t(1:H),sum(PV(:,1:H)),'y','LineWidth',1.8)
title('Total Load & PV')
legend('Load','PV')

% ---- SOC
subplot(4,2,2)
plot(t(1:H),SOC(1:H),'b','LineWidth',2)
ylim([0 1])
title('Battery SOC')

% ---- Import/Export
subplot(4,2,3)
plot(t(1:H),Pimp_hist(1:H),'r','LineWidth',1.6); hold on
plot(t(1:H),sum(Pexp_hist(:,1:H),1),'g','LineWidth',1.6)
title('Grid Power')
legend('Import','Export')

% ---- Charge/Discharge
subplot(4,2,4)
plot(t(1:H),Pch_hist(1:H),'b'); hold on
plot(t(1:H),Pdis_hist(1:H),'m')
title('Battery Power')
legend('Charge','Discharge')

% ---- Curtailment total
subplot(4,2,5)
plot(t(1:H),sum(Curt_hist(:,1:H),1),'k','LineWidth',1.6)
title('Total Curtailment')

% ---- Tariffs
subplot(4,2,6)
plot(t(1:H),T_imp(1:H),'LineWidth',2)
title('Import Tariff')

% ---- Voltage (CLEAR version)
subplot(4,2,7)
plot(t(1:H),V(:,1:H)')
hold on
yline(1.05,'r--'); yline(0.95,'r--');
title('Pole Voltages')
xlabel('Hour')

% ---- Cost per hour
subplot(4,2,8)
hour_cost = T_imp(1:H).*Pimp_hist(1:H)*dt;
plot(t(1:H),hour_cost,'LineWidth',1.6)
title('Hourly Import Cost')

sgtitle('Two-Layer EMS MPC Results')


