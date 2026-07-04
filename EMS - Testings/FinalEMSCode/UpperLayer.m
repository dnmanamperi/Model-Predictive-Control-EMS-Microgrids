%% =========================================================
% UPPER LAYER EXPORT LIMIT OPTIMIZATION
%% =========================================================

clear; clc;

Npoles = 14;

%% Slack voltage
Vslack = 1.0;

%% Example data (replace with your forecast at time t)
Pload = 3 + rand(1,Npoles);        % kW
PVavail = 5 + 5*rand(1,Npoles);    % kW

%% Sensitivity matrix (your method)
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

%% Convert to pu/kW
Vbase = 400;
Sbase = 100e3;
Zbase = Vbase^2 / Sbase;
S = S / Zbase;

%% =========================================================
% LP FORMULATION
%% =========================================================

% Objective: maximize sum(x)
% linprog does minimization → minimize -sum(x)

f = -ones(Npoles,1);

%% Voltage constraints

% Upper bound: V <= 1.05
% Vslack + S*(x - Pload') <= 1.05

A1 = S;
b1 = 1.05 - Vslack + S*Pload';

% Lower bound: V >= 0.95
% Vslack + S*(x - Pload') >= 0.95
% → -Sx <= -(0.95 - Vslack + S*Pload')

A2 = -S;
b2 = -(0.95 - Vslack + S*Pload');

A = [A1; A2];
b = [b1; b2];

%% Bounds on x
lb = zeros(Npoles,1);
ub = PVavail';

%% Solve LP
options = optimoptions('linprog','Display','none');
[x_opt, fval] = linprog(f, A, b, [], [], lb, ub, options);

%% Results
disp('Optimal Export Limits per Pole (kW):')
disp(x_opt')

disp('Total Hosting Capacity (kW):')
disp(sum(x_opt))

%% Compute resulting voltages
V = Vslack + S*(x_opt - Pload');
disp('Resulting Voltages (pu):')
disp(V')


%% =========================================================
% PLOT 1: EXPORT LIMITS
%% =========================================================
figure;
bar(x_opt,'LineWidth',1.2);
xlabel('Pole Number');
ylabel('Export Limit (kW)');
title('Optimal PV Export Limits per Pole');
grid on;

%% =========================================================
% PLOT 2: VOLTAGE PROFILE
%% =========================================================
figure;
plot(1:Npoles, V, 'o-','LineWidth',1.5);
hold on;
yline(1.05,'r--','Upper Limit 1.05 pu');
yline(0.95,'r--','Lower Limit 0.95 pu');
xlabel('Pole Number');
ylabel('Voltage (pu)');
title('Voltage Profile After Optimization');
grid on;


%% =========================================================
% PLOT 3: AVAILABLE vs ALLOWED PV
%% =========================================================
figure;
bar([PVavail' x_opt],'grouped');
xlabel('Pole Number');
ylabel('Power (kW)');
legend('Available PV','Allowed Export');
title('PV Curtailment Due to Voltage Constraints');
grid on;


%% =========================================================
% Compare With No Export Limit
%% =========================================================
V_uncontrolled = Vslack + S*(PVavail' - Pload');

figure;
plot(1:Npoles, V_uncontrolled, 'r--','LineWidth',1.2);
hold on;
plot(1:Npoles, V, 'b-o','LineWidth',1.5);
yline(1.05,'k--');
xlabel('Pole Number');
ylabel('Voltage (pu)');
legend('Uncontrolled','Optimized','Limit 1.05');
title('Voltage With and Without Export Control');
grid on;