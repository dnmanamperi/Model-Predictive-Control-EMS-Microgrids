%% =========================================================
% Dual-Layer EMS (Upper tariffs + Lower MPC dispatch) — v4
% - Upper export tariff: QP design of X_k(t) and mapping to c_exp,k(t)
% - Upper import tariff: flat daily adder φ for battery cost recovery
% - Lower layer: MPC (24h horizon), minimize operating cost incl. curtailment
%
% Voltage model (paper-style, drop from slack) with V0=400V:
%   ΔV_pu = (1000/V0^2) * R(Ω) * P(kW)
%   V_pu  = 1 - ΔV_pu   (net load P>0). Export behaves like negative load.
%
% This script also plots: V_noexp, V_allSurplus, V_qp, V_mpc.
%% =========================================================
clc; clear; close all;

%% -------------------------
% User settings
%% -------------------------
dt   = 0.25;          % hours (15 min)
Nh   = 96;            % MPC horizon (24h)
Nrun = 96;            % simulate 1 day (first 96 steps)
K    = 14;

% Voltage bounds
Vmin = 0.95;
Vmax = 1.05;

% Voltage base (given requirement)
Vbase = 400;          % volts

% If your R matrix is already pu/kW (NOT ohms), set true:
USE_R_AS_PU = false;

% Battery (100 kWh)
Ebat   = 100;         % kWh
SOCmin = 0.20*Ebat;
SOCmax = 0.80*Ebat;
SOC0   = 0.50*Ebat;

Pbat_max = 50;        % kW (tune)
eta_ch  = 0.95;
eta_dis = 0.95;

% Grid limits (optional)
Pimp_max = 1e6;       % kW
Pexp_max = 1e6;       % kW total export limit (optional)

% Import base tariff
%c_imp_base = 30;      % LKR/kWh (replace with TOU if needed)

% Export tariff bounds (QP mapping)
c_exp_min = 15;        % LKR/kWh
c_exp_max = 25;       % LKR/kWh

% Curtailment penalty
c_curt = 1;           % LKR/kWh curtailed

% Export QP weights
wV = 1.0;             % voltage weight (scalar or Kx1)
lambda = 1.0;         % curtailment-reduction weight

% Battery economics (ASSUMED defaults; tune)
econ.capex_per_kWh_LKR = 120000;  % LKR/kWh
econ.life_years = 15;
econ.discount_rate = 0.10;
econ.fixed_om_frac = 0.015;
econ.cycle_life = 4000;
econ.DoD = 0.60;
econ.deg_cost_factor = 0.20;

% Numeric cleanup threshold for plots
EPS_CLEAN = 1e-6;

%% -------------------------
% Excel inputs
%% -------------------------
loadPV_xlsx  = "afterforecasting.xlsx";
loadPV_sheet = "Sheet1";


%% =========================================================
% Read Load & PV
%% =========================================================
[Load_TxK, PV_TxK, time_str] = read_load_pv_from_excel(loadPV_xlsx, loadPV_sheet, 15, datetime(2025,7,10,0,0,0));
T = size(Load_TxK,1);

needT = Nh + Nrun - 1;
if T < needT
    error("Need at least %d timesteps, got %d", needT, T);
end

Load = Load_TxK(:,1:K)';   % K x T
PV   = PV_TxK(:,1:K)';     % K x T

time_plot = (0:Nrun-1)*dt;

% PV pole mapping sanity check
pv_poles = find(max(PV(:,1:needT),[],2) > 1e-6)';
disp('Poles with non-zero PV (expected [2 5 7 9 10 14]):');
disp(pv_poles);

%% =========================================================
% Build sensitivity matrix S internally (from line resistances)
%% =========================================================
r_per_km = 0.443;
dist_m = [23.35 20.42 20.09 21.74 23.76 24.06 22 21.4 ...
          32.07 24.19 13.57 21.79 28.15 19.78];

dist_km = dist_m/1000;
Rseg = r_per_km * dist_km;   % segment resistances (ohm)

c = cumsum(Rseg);            % cumulative resistance to each node (ohm)

S = zeros(K);
for i = 1:K
    for j = 1:K
        S(i,j) = c(min(i,j));
    end
end
S = S*3;
% Convert to pu/kW sensitivity
if USE_R_AS_PU
    Rpu = S;  % already pu/kW (unlikely here)
else
    Rpu = (1000/(Vbase^2)) * S;
end

%% =========================================================
% PV self-consumption + surplus
%% =========================================================
PVself0 = min(PV, Load);
PVsur   = max(PV - Load, 0);     % surplus PV for export/curtail/charge
Pnet_self = Load - PVself0;      % net load after self

% Baseline voltage with no export (pu)
V0 = zeros(K,T);
for t=1:T
    V0(:,t) = 1 - Rpu * Pnet_self(:,t);
end

%% =========================================================
% Upper layer: export tariff QP
% V = V0 + Rpu * X
%% =========================================================
qp_in.V0     = V0(:,1:needT);
qp_in.S      = Rpu;
qp_in.PVmax  = PVsur(:,1:needT);
qp_in.Vmin   = Vmin;
qp_in.Vmax   = Vmax;
qp_in.wV     = wV;
qp_in.lambda = lambda;
qp_in.cmin   = c_exp_min;
qp_in.cmax   = c_exp_max;

[Xcap, c_exp_pole] = export_tariff_qp_design(qp_in);

%% =========================================================
% Battery economics -> c_deg + daily fixed cost; compute phi
%% =========================================================
[c_deg, C_fixed_day] = compute_battery_costs(econ, Ebat);

fprintf("\n=== Battery economics (assumed) ===\n");
fprintf("c_deg = %.4f LKR/kWh-throughput\n", c_deg);
fprintf("Fixed daily cost = %.2f LKR/day\n", C_fixed_day);

% Build TOU baseline import tariff (LKR/kWh)
c_imp_vec = build_CEB_TOU_import_tariff(dt, needT);

figure('Name','CEB TOU Import Tariff (Baseline)','Position',[200 200 900 300])
plot((0:Nrun-1)*dt, c_imp_vec(1:Nrun), 'LineWidth', 1.4); grid on
xlabel('Hour'); ylabel('LKR/kWh'); title('Baseline Import Tariff (TOU)')

mpc.K=K; mpc.dt=dt; mpc.Nh=Nh; mpc.Nrun=Nrun;
mpc.Vmin=Vmin; mpc.Vmax=Vmax; mpc.Rpu=Rpu;
mpc.SOC0=SOC0; mpc.SOCmin=SOCmin; mpc.SOCmax=SOCmax;
mpc.Pbat_max=Pbat_max; mpc.eta_ch=eta_ch; mpc.eta_dis=eta_dis;
mpc.Pimp_max=Pimp_max; mpc.Pgrid_exp_max=Pexp_max;  % PCC export cap
mpc.c_deg=c_deg; mpc.c_curt=c_curt;

% Baseline run (phi=0) to estimate E_import & degradation
base = run_lower_layer_mpc(Load, PV, PVsur, Pnet_self, Xcap, c_exp_pole, c_imp_vec, mpc, 0);

E_imp_hat = sum(base.Pimp)*dt;
C_deg_hat = base.Cdeg_total;
R_target_day = C_fixed_day + C_deg_hat;

if E_imp_hat <= 1e-9
    phi = 0;
else
    phi = R_target_day / E_imp_hat;
end

fprintf("\n=== Flat import adder ===\n");
fprintf("E_import_hat = %.3f kWh\n", E_imp_hat);
fprintf("C_deg_hat    = %.2f LKR\n", C_deg_hat);
fprintf("R_target_day = %.2f LKR/day\n", R_target_day);
fprintf("phi          = %.6f LKR/kWh\n", phi);

%% =========================================================
% Final MPC with phi
%% =========================================================
out = run_lower_layer_mpc(Load, PV, PVsur, Pnet_self, Xcap, c_exp_pole, c_imp_vec, mpc, phi);

%% =========================================================
% Outputs
%% =========================================================
Pimp = out.Pimp;
Pch  = out.Pch;
Pdis = out.Pdis;
SOC  = out.SOC;
SOC_pct = 100*SOC/Ebat;

Ppay_pole = out.Ppay_pole;   % paid export
Pinj_pole = out.Pinj_pole;       % physical injection
Pgrid_exp = out.Pgrid_exp;       % PCC export (unpaid)

Curt_pole = out.Curt_pole;

%% ==============================
% Physical injection + PCC import/export
%% ==============================

% Physical injection is already provided by lower layer:
%   Pinj_pole(:,t) = PVsur(:,t) - Curt_pole(:,t)  (kW)
Pinj_total = sum(Pinj_pole, 1)';       % Nrun x 1 (kW)

% PCC import/export from lower-layer variables
P_grid_import = Pimp;                 % nonnegative by definition
P_grid_export = Pgrid_exp;            % nonnegative by definition

E_grid_import = sum(P_grid_import) * dt;
E_grid_export = sum(P_grid_export) * dt;

fprintf('\n=== PCC / GRID FLOW SUMMARY ===\n');
fprintf('Grid import energy  = %.2f kWh\n', E_grid_import);
fprintf('Grid export energy  = %.2f kWh\n', E_grid_export);
fprintf('===============================\n');

Ppay = sum(Ppay_pole,1)';
Curt = sum(Curt_pole,1)';

V_mpc = out.V_real;
total_cost = out.total_cost;

Load_tot = sum(Load(:,1:Nrun),1);
PV_tot   = sum(PV(:,1:Nrun),1);
Pbat     = (Pdis - Pch);

%% Metrics
E_import = sum(Pimp)*dt;
E_export_paid = sum(Ppay)*dt;
E_curt   = sum(Curt)*dt;
E_pv     = sum(PV_tot)*dt;
PV_util  = 1 - E_curt/max(E_pv,1e-9);

fprintf('\n============= PERFORMANCE METRICS =============\n');
fprintf('Total cost             = %.2f LKR\n', total_cost);
fprintf('phi (flat adder)       = %.6f LKR/kWh\n', phi);
fprintf('Import energy          = %.2f kWh\n', E_import);
fprintf('Paid export energy     = %.2f kWh\n', E_export_paid);
fprintf('Curtailment energy     = %.6f kWh\n', E_curt);
fprintf('PV utilization         = %.2f %%\n', 100*PV_util);
fprintf('===============================================\n');

%% =========================================================
% Voltage comparison curves
%% =========================================================
V_noexp = V0(:,1:Nrun);
V_all   = V0(:,1:Nrun) + Rpu*PVsur(:,1:Nrun);    % export ALL surplus
V_qp    = V0(:,1:Nrun) + Rpu*Xcap(:,1:Nrun);     % QP-capped export

%% =========================================================
% Plots (reference style)
%% =========================================================
figure('Name','Feeder Overview','Position',[100 80 1200 900])
subplot(5,1,1)
plot(time_plot, Load_tot, 'k', time_plot, PV_tot, 'g', 'LineWidth',1.4)
legend('Load','PV'); title('Load & PV'); grid on

subplot(5,1,2)
plot(time_plot, Pimp,'b','LineWidth',1.2); hold on;
plot(time_plot, sum(Ppay_pole,1)','r','LineWidth',1.2);
title('Grid Import vs Total PAID Pole Export'); grid on
legend('P_{imp}','\Sigma P_{pay,pole}');

subplot(5,1,3)
plot(time_plot, Pbat,'LineWidth',1.2); yline(0);
title('Battery Power (Pdis-Pch)'); grid on

subplot(5,1,4)
plot(time_plot, SOC_pct,'LineWidth',1.2); ylim([0 100]);
title('SOC (%)'); grid on

subplot(5,1,5)
plot(time_plot, Curt,'LineWidth',1.2);
title('Curtailment (Total)'); grid on
xlabel('Hour');

figure('Name','Pole-wise Curtailment','Position',[100 50 1200 900])
for p=1:K
    subplot(4,4,p)
    plot(time_plot, 1000*Curt_pole(p,:),'r','LineWidth',1.1);
    title(['Pole ' num2str(p)]); xlabel('Hour'); ylabel('W'); grid on
end
sgtitle('Pole-wise Curtailment');

figure('Name','Pole-wise Injection to Feeder','Position',[100 50 1200 900])
for p = 1:K
    subplot(4,4,p)
    plot(time_plot, Pinj_pole(p,:), 'b', 'LineWidth', 1.1);
    title(['Pole ' num2str(p)])
    xlabel('Hour'); ylabel('W'); grid on
end
sgtitle('Pole-wise Injection to Feeder (Pinj)');

figure('Name','PCC Import/Export to Grid','Position',[150 150 1200 350])
plot(time_plot, P_grid_import, 'LineWidth', 1.3); hold on
plot(time_plot, P_grid_export, 'LineWidth', 1.3);
grid on
xlabel('Hour'); ylabel('W');
title('Grid Import and Grid Export at PCC');
legend('Grid Import','Grid Export');

figure('Name','Pole-wise Export Tariff (from QP)','Position',[100 50 1200 900])
for p=1:K
    subplot(4,4,p)
    plot(time_plot, c_exp_pole(p,1:Nrun),'m','LineWidth',1.1);
    title(['Pole ' num2str(p)]); xlabel('Hour'); ylabel('LKR/kWh'); grid on
end
sgtitle('Pole-wise Export Tariff');

figure('Name','Upper-layer QP Export Caps X_k(t)','Position',[100 50 1200 900])
for p=1:K
    subplot(4,4,p)
    plot(time_plot, Xcap(p,1:Nrun),'LineWidth',1.1);
    title(['Pole ' num2str(p)]); xlabel('Hour'); ylabel('kW'); grid on
end
sgtitle('Upper-layer QP: X_k(t) (export caps)');

figure('Name','Pole-wise Voltage (MPC realized, pu)','Position',[100 50 1200 900])
for p=1:K
    subplot(4,4,p)
    plot(time_plot, V_mpc(p,:),'LineWidth',1.1); hold on;
    yline(1,'--k'); yline(Vmin,':k'); yline(Vmax,':k');
    title(['Pole ' num2str(p)]); xlabel('Hour'); ylabel('p.u.'); grid on
end
sgtitle('Pole-wise Voltage (MPC)');

% Voltage comparison plot (PV poles)
pv_poles = find(max(PV(:,1:Nrun),[],2) > 1e-6);
if ~isempty(pv_poles)
    figure('Name','Voltage Comparison (NoExp vs AllSurplus vs QP vs MPC)','Position',[100 50 1200 900])
    for ii=1:min(numel(pv_poles),12)
        p = pv_poles(ii);
        subplot(3,4,ii)
        plot(time_plot, V_noexp(p,:),'k','LineWidth',1.1); hold on
        plot(time_plot, V_all(p,:),'r','LineWidth',1.1);
        plot(time_plot, V_qp(p,:),'b','LineWidth',1.1);
        plot(time_plot, V_mpc(p,:),'g--','LineWidth',1.1);
        yline(1,'--'); yline(Vmin,':'); yline(Vmax,':');
        title(['Pole ' num2str(p)]); grid on
    end
    legend('No export','All surplus export','QP predicted','MPC realized');
end

% Voltage in volts
V_volts = V_mpc * Vbase;
figure('Name','Pole-wise Voltage (Volts)','Position',[100 50 1200 900])
for p=1:K
    subplot(4,4,p)
    plot(time_plot, V_volts(p,:),'LineWidth',1.1); hold on;
    yline(Vbase,'--k'); yline(Vmin*Vbase,':k'); yline(Vmax*Vbase,':k');
    title(['Pole ' num2str(p)]); xlabel('Hour'); ylabel('V'); grid on
end
sgtitle('Pole-wise Voltage (V)');