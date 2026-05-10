%% =========================================================
%  TWO-LAYER EMS (FULLY UPDATED) + TRUE MPC + ECON-BASED BATTERY COSTING
%
%  UPDATES IN THIS VERSION:
%   1) Import tariff is TOU (no flat import tariff anymore):
%        Off-peak (22:30–05:30): 21 LKR/kWh
%        Day      (05:30–18:30): 35 LKR/kWh
%        Peak     (18:30–22:30): 67 LKR/kWh
%   2) Removed phi_max clamp (no max limit on ϕ)
%   3) Added curtailment penalty to LL objective:
%        + c_curt * sum_k P_pv_curt(k,t) * dt
%   4) SOC dynamics sign FIX in LP (prevents infeasibility due to drift)
%   5) Prints Performance Metrics block at end
%   6) Battery total CAPEX fixed to 4,800,000 LKR (100 kWh)
%% =========================================================
clc; clear; close all;

%% =====================
% READ EXCEL DATA
%% =====================
xlsxPath = 'Reconstructed_14Pole_Load_Solar.xlsx';
T = readtable(xlsxPath);

Npoles = 14;
dt = 0.25; % hours (15 min)

Nsim = height(T);
Load = zeros(Npoles, Nsim);
PVmax = zeros(Npoles, Nsim);

for p = 1:Npoles
    Load(p,:)  = T.(sprintf('Pole_%d_Load_kW',  p))';
    PVmax(p,:) = T.(sprintf('Pole_%d_Solar_kW', p))';
end

time = (0:Nsim-1)*dt;   % hours from 0

%% =====================
% BATTERY @ PCC (100 kWh)
%% =====================
Ebat    = 100;         % kWh
SOCmin  = 0.20*Ebat;
SOCmax  = 0.80*Ebat;
SOC0    = 0.50*Ebat;

PbatMax = 200;         % kW (tune)
eta_ch  = 0.95;
eta_dis = 0.95;

%% =====================
% GRID LIMITS @ PCC
%% =====================
PimpMax = 1e6;         % kW
PexpMax = 1e6;         % kW

%% =====================
% EXPORT TARIFF BOUNDS + CURTAILMENT PENALTY
%% =====================
c_min  = 15;           % LKR/kWh
c_max  = 25;           % LKR/kWh
c_curt = 1;            % LKR/kWh curtailed  <<< ADDED (tune if needed)

%% =====================
% IMPORT TARIFF (TOU)  <<< UPDATED (NO FLAT TARIFF)
%% =====================
% Time-of-day tariff (LKR/kWh):
% Off-peak (22:30–05:30): 21
% Day      (05:30–18:30): 35
% Peak     (18:30–22:30): 67
c_imp = build_TOU_tariff(time, 21, 35, 67);  % length Nsim

%% =====================
% BATTERY ECONOMICS (TOTAL CAPEX = 4,800,000 LKR)
%% =====================
econ.capex_per_kWh_LKR = 4800000 / Ebat;   % = 48000 LKR/kWh for 100 kWh
econ.life_years = 10;
econ.discount_rate = 0.10;
econ.fixed_om_frac = 0.015;
econ.cycle_life = 4000;
econ.DoD = 0.60;
econ.deg_cost_factor = 0.20;

[c_deg, C_fixed_day] = compute_battery_costs(econ, Ebat);

R_target = C_fixed_day;  % LKR/day recovered via import adder (ϕ)
capex_total = econ.capex_per_kWh_LKR * Ebat;

fprintf('Battery economics:\n');
fprintf('  Ebat (kWh)              = %.2f\n', Ebat);
fprintf('  Total CAPEX (LKR)       = %.2f\n', capex_total);
fprintf('  CAPEX per kWh (LKR/kWh) = %.2f\n', econ.capex_per_kWh_LKR);
fprintf('  c_deg (LKR/kWh-through) = %.6f\n', c_deg);
fprintf('  C_fixed_day (LKR/day)   = %.2f\n\n', C_fixed_day);

%% =====================
% VOLTAGE MODEL SETTINGS
%% =====================
Vref = 1.0;
Vmin = 0.95;
Vmax = 1.05;

% -------- Sensitivity S (shared-path resistance) ----------
r_per_km = 0.443;
dist_m = [23.35 20.42 20.09 21.74 23.76 24.06 22 21.4 ...
          32.07 24.19 13.57 21.79 28.15 19.78];

dist_km = dist_m/1000;
Rseg_ohm = r_per_km * dist_km;
cseg = cumsum(Rseg_ohm);

S_ohm = zeros(Npoles);
for i = 1:Npoles
    for j = 1:Npoles
        S_ohm(i,j) = cseg(min(i,j));
    end
end

VLL = 400;
S = (1000/(VLL^2)) * S_ohm;   % pu per kW

% NOTE: if sign convention differs, flip:
% S = -S;

% -------- Base voltage profile V0 (p.u.) ----------
V0 = ones(Npoles, Nsim);
V0 = V0 - 0.005 * repmat((0:Npoles-1)', 1, Nsim)/Npoles;

%% =====================
% MPC SETTINGS
%% =====================
Nh  = 96;
Nrt = Nsim;

optionsQP = optimoptions('quadprog','Display','none');
optionsLP = optimoptions('linprog', ...
    'Algorithm','dual-simplex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-8, ...
    'OptimalityTolerance',1e-8);

%% =====================
% STORAGE
%% =====================
Pimp    = zeros(Nrt,1);
PexpPCC = zeros(Nrt,1);
Pch     = zeros(Nrt,1);
Pdis    = zeros(Nrt,1);
SOC     = zeros(Nrt,1);

Pself   = zeros(Npoles, Nrt);     % P_pv_self
Pexp_k  = zeros(Npoles, Nrt);     % P_pv_exp
Pcurt   = zeros(Npoles, Nrt);     % P_pv_curt

Xcap    = zeros(Npoles, Nrt);     % X_k(t)
c_exp_k = zeros(Npoles, Nrt);     % c_exp,k(t) at applied step

V_QP    = zeros(Npoles, Nrt);
V_MPC   = zeros(Npoles, Nrt);

phi_rt  = zeros(Nrt,1);

SOC_now = SOC0;

%% =========================================================
% TRUE MPC LOOP
%% =========================================================
for k = 1:Nrt

    % Optional safety clamp
    SOC_now = min(max(SOC_now, SOCmin), SOCmax);

    Hleft = min(Nh, Nsim - k + 1);
    idx0  = k;
    idxH  = k + Hleft - 1;

    Load_f  = Load(:, idx0:idxH);
    PV_f    = PVmax(:, idx0:idxH);
    V0_f    = V0(:, idx0:idxH);
    cimp_f  = c_imp(idx0:idxH);          % TOU tariff horizon

    %% =============================
    % UL-1: phi = R_target / E_imp_hat   (NO MAX LIMIT)
    % Baseline import estimate: net deficit ignoring battery + incentives
    %% =============================
    net_def   = max(sum(Load_f,1) - sum(PV_f,1), 0);   % kW
    E_imp_hat = sum(net_def) * dt;                     % kWh
    if E_imp_hat < 1e-6
        phi = 0;
    else
        phi = R_target / E_imp_hat;                    % NO CLAMP
    end
    phi_rt(k) = phi;

    %% =============================
    % UL-2: QP for Xcap(t)
    %% =============================
    lambda  = 1.0;       % λ >= 0 (curtailment reduction weight)
    reg_eps = 1e-6;

    w = ones(Npoles,1);  % w_i >= 0 (node weights)
    W = diag(w);

    Xcap_h = zeros(Npoles, Hleft);
    Vqp_h  = zeros(Npoles, Hleft);

    for i = 1:Hleft
        Vb   = V0_f(:,i);
        pv_i = PV_f(:,i);

        Hq = 2*(S' * W * S) + 2*reg_eps*eye(Npoles);
        fq = 2*(S' * W * (Vb - Vref*ones(Npoles,1))) - lambda*ones(Npoles,1);

        lbx = zeros(Npoles,1);
        ubx = pv_i;

        A  = [ S; -S ];
        b  = [ Vmax*ones(Npoles,1) - Vb;
              -(Vmin*ones(Npoles,1) - Vb) ];

        xcap = quadprog(Hq, fq, A, b, [], [], lbx, ubx, [], optionsQP);
        if isempty(xcap), xcap = zeros(Npoles,1); end

        Xcap_h(:,i) = xcap;
        Vqp_h(:,i)  = Vb + S*xcap;
    end

    Xcap(:,k) = Xcap_h(:,1);
    V_QP(:,k) = Vqp_h(:,1);

    %% =============================
    % UL-3: c_exp,k(t) mapping from Xcap/PVmax
    %% =============================
    pv_now = PV_f(:,1);
    ratio = zeros(Npoles,1);
    nz = pv_now > 1e-6;
    ratio(nz) = Xcap_h(nz,1) ./ pv_now(nz);
    ratio = max(0, min(1, ratio));
    c_exp_k(:,k) = c_min + (c_max - c_min)*ratio;

    %% =============================
    % LOWER LAYER MPC (LP)
    %% =============================
    nv   = 4 + Npoles + Npoles + Npoles + 1;
    nvar = Hleft*nv;

    iPimp   = 1;  iPexpPCC = 2;  iPch = 3;  iPdis = 4;
    iPself0 = 5;
    iPexp0  = 5 + Npoles;
    iPcurt0 = 5 + 2*Npoles;
    iSOC    = 5 + 3*Npoles;

    % Objective (NOW includes curtailment penalty)
    f = zeros(nvar,1);
    for i = 1:Hleft
        off = (i-1)*nv;

        % Import energy cost: (c_imp(t) + phi) * Pimp
        f(off+iPimp) = (cimp_f(i) + phi)*dt;

        % Battery degradation (throughput)
        f(off+iPch)  = c_deg*dt;
        f(off+iPdis) = c_deg*dt;

        % Curtailment penalty: + c_curt * sum_k Pcurt_k
        f(off+iPcurt0:off+iPcurt0+Npoles-1) = c_curt*dt;

        % Pole-wise export revenue: - sum_k cexp_k(t)*Ppv_exp_k(t)
        pv_i = PV_f(:,i);
        ratio_i = zeros(Npoles,1);
        nz2 = pv_i > 1e-6;
        ratio_i(nz2) = Xcap_h(nz2,i) ./ pv_i(nz2);
        ratio_i = max(0, min(1, ratio_i));
        cexp_i = c_min + (c_max - c_min)*ratio_i;

        f(off+iPexp0:off+iPexp0+Npoles-1) = -cexp_i*dt;
    end

    % Bounds
    lb = zeros(nvar,1);
    ub = inf(nvar,1);

    for i = 1:Hleft
        off = (i-1)*nv;

        ub(off+iPimp)    = PimpMax;
        ub(off+iPexpPCC) = PexpMax;
        ub(off+iPch)     = PbatMax;
        ub(off+iPdis)    = PbatMax;

        ub(off+iPself0:off+iPself0+Npoles-1) = Load_f(:,i);
        ub(off+iPexp0:off+iPexp0+Npoles-1)   = Xcap_h(:,i);
        ub(off+iPcurt0:off+iPcurt0+Npoles-1) = PV_f(:,i);

        lb(off+iSOC) = SOCmin;
        ub(off+iSOC) = SOCmax;
    end

    % Equalities
    Aeq = [];
    beq = [];

    % (1) PV split: Pself + Pexp + Pcurt = PVmax
    for i = 1:Hleft
        off = (i-1)*nv;
        for p = 1:Npoles
            row = zeros(1,nvar);
            row(off+iPself0+p-1) = 1;
            row(off+iPexp0 +p-1) = 1;
            row(off+iPcurt0+p-1) = 1;
            Aeq = [Aeq; row];
            beq = [beq; PV_f(p,i)];
        end
    end

    % (2) Feeder balance:
    % Pimp - Pexp_grid - Pch + Pdis = sum(Load) - sum(PVmax) + sum(Pcurt)
    for i = 1:Hleft
        off = (i-1)*nv;
        row = zeros(1,nvar);
        row(off+iPimp)    = 1;
        row(off+iPexpPCC) = -1;
        row(off+iPch)     = -1;
        row(off+iPdis)    = 1;
        row(off+iPcurt0:off+iPcurt0+Npoles-1) = 1;
        Aeq = [Aeq; row];
        beq = [beq; sum(Load_f(:,i)) - sum(PV_f(:,i))];
    end

    % (3) SOC dynamics (CORRECT SIGNS):
    % SOC(i+1) = SOC(i) + dt*(eta_ch*Pch - (1/eta_dis)*Pdis)
    for i = 1:Hleft-1
        off  = (i-1)*nv;
        off2 = i*nv;
        row = zeros(1,nvar);
        row(off+iSOC)  = 1;
        row(off+iPch)  =  dt*eta_ch;
        row(off+iPdis) = -dt*(1/eta_dis);
        row(off2+iSOC) = -1;
        Aeq = [Aeq; row];
        beq = [beq; 0];
    end

    % (4) Initial SOC
    row0 = zeros(1,nvar);
    row0(iSOC) = 1;
    Aeq = [Aeq; row0];
    beq = [beq; SOC_now];

    % Solve LP
    [x, fval, exitflag, output] = linprog(f, [], [], Aeq, beq, lb, ub, optionsLP);
    if exitflag <= 0 || isempty(x)
        warning('Lower MPC failed at k=%d | exitflag=%d | %s', k, exitflag, output.message);
        fprintf('  Hleft=%d, nvar=%d, #eq=%d\n', Hleft, nvar, size(Aeq,1));
        fprintf('  SOC_now=%.3f (bounds [%.3f, %.3f])\n', SOC_now, SOCmin, SOCmax);
        fprintf('  sumLoad_now=%.3f kW, sumPVmax_now=%.3f kW\n', sum(Load_f(:,1)), sum(PV_f(:,1)));
        fprintf('  min(Xcap_now)=%.3f, max(Xcap_now)=%.3f\n', min(Xcap_h(:,1)), max(Xcap_h(:,1)));
        break;
    end

    X = reshape(x, nv, Hleft)';

    % Apply first step
    Pimp(k)    = X(1,iPimp);
    PexpPCC(k) = X(1,iPexpPCC);
    Pch(k)     = X(1,iPch);
    Pdis(k)    = X(1,iPdis);

    Pself(:,k) = X(1, iPself0:iPself0+Npoles-1)';
    Pexp_k(:,k)= X(1, iPexp0:iPexp0+Npoles-1)';
    Pcurt(:,k) = X(1, iPcurt0:iPcurt0+Npoles-1)';

    % True SOC update
    SOC_now = SOC_now + dt*(eta_ch*Pch(k) - (1/eta_dis)*Pdis(k));
    SOC(k)  = SOC_now;

    % Voltage realized from pole exports
    V_MPC(:,k) = V0(:,idx0) + S*Pexp_k(:,k);
end

%% =====================
% PERFORMANCE METRICS SUMMARY
%% =====================

% Export tariff time-series (consistent with UL-3)
c_exp_real = zeros(Npoles, Nrt);
for t = 1:Nrt
    pv_t = PVmax(:,t);
    ratio_t = zeros(Npoles,1);
    nz = pv_t > 1e-6;
    ratio_t(nz) = Xcap(nz,t) ./ pv_t(nz);
    ratio_t = max(0, min(1, ratio_t));
    c_exp_real(:,t) = c_min + (c_max - c_min)*ratio_t;
end

E_imp   = sum(Pimp)    * dt;
E_exp   = sum(PexpPCC) * dt;
E_curt  = sum(sum(Pcurt,1)) * dt;
E_pv_av = sum(sum(PVmax,1)) * dt;
pv_util = 100 * (1 - E_curt / max(E_pv_av, 1e-9));

% Import cost uses TOU c_imp(t) + phi(t)
C_imp = sum((c_imp + phi_rt) .* Pimp) * dt;

% Pole-wise export revenue
Rev_exp = 0;
for t = 1:Nrt
    Rev_exp = Rev_exp + sum(c_exp_real(:,t) .* Pexp_k(:,t)) * dt;
end

% Degradation cost
C_deg = sum(Pch + Pdis) * c_deg * dt;

% Curtailment penalty
C_curt = E_curt * c_curt;

Total_cost = C_imp - Rev_exp + C_deg + C_curt;

fprintf('\n================= PERFORMANCE METRICS =================\n');
fprintf('Total cost            = %.2f  LKR\n', Total_cost);
fprintf('phi (flat adder)      = %.6f  LKR/kWh (mean)\n', mean(phi_rt(~isnan(phi_rt))));
fprintf('Import energy         = %.2f  kWh\n', E_imp);
fprintf('Export energy         = %.2f  kWh\n', E_exp);
fprintf('Curtailment energy    = %.2f  kWh\n', E_curt);
fprintf('PV utilization        = %.2f  %%\n', pv_util);
fprintf('=======================================================\n\n');

%% =====================
% PLOTS
%% =====================
SOC_pct = 100*SOC/Ebat;

figure('Name','Summary','Position',[80 60 950 850]);
subplot(5,1,1)
plot(time, sum(Load,1),'k','LineWidth',1.2); hold on;
plot(time, sum(PVmax,1),'g','LineWidth',1.2);
legend('Total Load','Total PVmax'); grid on; ylabel('kW'); title('Load & PV');

subplot(5,1,2)
plot(time, Pimp,'b','LineWidth',1.2); hold on;
plot(time, PexpPCC,'r','LineWidth',1.2);
legend('Grid Import','Grid Export (PCC)'); grid on; ylabel('kW'); title('PCC Import/Export');

subplot(5,1,3)
plot(time, (Pdis-Pch),'LineWidth',1.2); yline(0,'--');
grid on; ylabel('kW'); title('Battery Power (Pdis - Pch)');

subplot(5,1,4)
plot(time, SOC_pct,'LineWidth',1.2); grid on; ylim([0 100]);
ylabel('%'); title('SOC (%)');

subplot(5,1,5)
plot(time, sum(Pcurt,1),'LineWidth',1.2); grid on;
ylabel('kW'); xlabel('Hour'); title('Total Curtailment');

figure('Name','Pole-wise PV Export vs Caps','Position',[80 60 1200 900]);
for p=1:Npoles
    subplot(4,4,p);
    plot(time, Pexp_k(p,:), 'b','LineWidth',1.2); hold on;
    plot(time, Xcap(p,:), 'k--','LineWidth',1.0);
    title(sprintf('Pole %d',p)); grid on; xlabel('Hour'); ylabel('kW');
    if p==1, legend('P_{pv,exp,k}','X_k (cap)'); end
end
sgtitle('Pole-wise PV Export / Injection to Feeder');

figure('Name','Pole-wise Curtailment','Position',[80 60 1200 900]);
for p=1:Npoles
    subplot(4,4,p);
    plot(time, Pcurt(p,:), 'r','LineWidth',1.2);
    title(sprintf('Pole %d',p)); grid on; xlabel('Hour'); ylabel('kW');
end
sgtitle('Pole-wise PV Curtailment');

figure('Name','Voltage: Upper QP vs Realized MPC','Position',[80 60 1200 900]);
for p=1:Npoles
    subplot(4,4,p);
    plot(time, V_QP(p,:), 'g--','LineWidth',1.2); hold on;
    plot(time, V_MPC(p,:), 'r','LineWidth',1.2);
    yline(Vref,'k--'); yline(Vmin,':'); yline(Vmax,':');
    title(sprintf('Pole %d',p)); grid on; xlabel('Hour'); ylabel('p.u.');
    ylim([0.94 1.06]);
    if p==1, legend('V_{QP}','V_{MPC}'); end
end
sgtitle('Voltage: QP Forecast vs MPC Realized');

figure('Name','phi over time','Position',[200 150 900 300]);
plot(time, phi_rt,'LineWidth',1.3); grid on;
xlabel('Hour'); ylabel('LKR/kWh'); title('\phi (import adder) from economics-based revenue target');

figure('Name','TOU Import Tariff','Position',[200 200 900 250]);
plot(time, c_imp, 'LineWidth', 1.3); grid on;
xlabel('Hour'); ylabel('LKR/kWh'); title('c_{imp}(t) Time-of-Use Tariff');

%% =========================================================
% Local functions (keep at end of script)
%% =========================================================
function c_imp = build_TOU_tariff(time_hours, c_off, c_day, c_peak)
% time_hours: 0..(N-1)*dt
% TOU:
% Off-peak 22:30–05:30  => [22.5,24) U [0,5.5)
% Day      05:30–18:30  => [5.5,18.5)
% Peak     18:30–22:30  => [18.5,22.5)

tod = mod(time_hours, 24);  % time of day in hours
c_imp = zeros(size(tod));

is_off  = (tod >= 22.5) | (tod < 5.5);
is_day  = (tod >= 5.5)  & (tod < 18.5);
is_peak = (tod >= 18.5) & (tod < 22.5);

c_imp(is_off)  = c_off;
c_imp(is_day)  = c_day;
c_imp(is_peak) = c_peak;
end

function [c_deg, C_fixed_day] = compute_battery_costs(econ, Ebat_kWh)
% Computes:
% - c_deg: degradation cost coefficient [LKR/kWh throughput]
% - C_fixed_day: daily fixed cost [LKR/day] from CAPEX annuity + fixed O&M

capex_total = econ.capex_per_kWh_LKR * Ebat_kWh;

r = econ.discount_rate;
n = econ.life_years;
CRF = r*(1+r)^n / ((1+r)^n - 1);

annual_annuity = capex_total * CRF;
annual_om      = capex_total * econ.fixed_om_frac;

C_fixed_day = (annual_annuity + annual_om) / 365;

% Lifetime throughput approximation
E_dis_life = econ.cycle_life * econ.DoD * Ebat_kWh;
E_through_life = 2 * E_dis_life; % charge+dis throughput

raw_c_deg = capex_total / max(E_through_life, 1e-9); % LKR/kWh throughput
c_deg = econ.deg_cost_factor * raw_c_deg;
end