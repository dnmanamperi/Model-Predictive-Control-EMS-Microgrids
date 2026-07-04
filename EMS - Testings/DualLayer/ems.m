clc; clear; close all;




















%% =========================================================
% SYSTEM SETTINGS
%% =========================================================

Np = 14;           % number of poles
Ts = 0.25;         % 15 min
H  = 96;           % 24h MPC horizon
Tsim = 96;         % simulate 1 day

eta = 0.95;
SOCmin = 0.1;
SOCmax = 0.9;
Pbat_max = 3;

Curt_cost = 2;
Deg_cost  = 0.02;

V0 = 1;
S  = 0.002;
Vmax = 1.05;

rng(1);

%% =========================================================
% SYNTHETIC REAL PROFILES (truth)
%% =========================================================

[load_real, solar_real] = synthetic_profiles(Np,Tsim);

%% =========================================================
% STORAGE INITIALIZATION
%% =========================================================

SOC = 0.5*ones(Np,1);

Pimp_hist  = zeros(Np,Tsim);
Pexp_hist  = zeros(Np,Tsim);
Pcurt_hist = zeros(Np,Tsim);
Pbat_hist  = zeros(Np,Tsim);
SOC_hist   = zeros(Np,Tsim);

%% =========================================================
% RECEDING HORIZON LOOP (REAL EMS BEHAVIOR)
%% =========================================================

for t = 1:Tsim

    fprintf("Step %d / %d\n",t,Tsim)

    %% 1) MEASURE REAL VALUES
    load_now  = load_real(:,t);
    solar_now = solar_real(:,t);

    %% 2) BUILD FORECAST (simple noisy persistence)
    loadF  = forecast_profile(load_real,t,H);
    solarF = forecast_profile(solar_real,t,H);

    %% 3) UPPER LAYER → dynamic tariffs
    [lambda_imp, lambda_exp] = upper_layer_tariff(loadF,solarF,S,V0,Vmax);

    %% 4) LOWER LAYER → MPC solve
    sol = lower_layer_mpc(loadF,solarF,lambda_imp,lambda_exp,...
                          SOC,Ts,eta,Pbat_max,SOCmin,SOCmax,...
                          Curt_cost,Deg_cost,S,V0,Vmax);

    %% 5) APPLY ONLY FIRST STEP
    Pimp  = sol.Pimp(:,1);
    Pexp  = sol.Pexp(:,1);
    Pcurt = sol.Pcurt(:,1);
    Pbat  = sol.Pbat(:,1);

    %% 6) UPDATE SOC USING REAL ACTION
    SOC = SOC + Ts*Pbat;

    %% 7) STORE
    Pimp_hist(:,t)  = Pimp;
    Pexp_hist(:,t)  = Pexp;
    Pcurt_hist(:,t) = Pcurt;
    Pbat_hist(:,t)  = Pbat;
    SOC_hist(:,t)   = SOC;
end

%% =========================================================
% RESULTS PLOT
%% =========================================================

figure
plot(sum(Pcurt_hist,1))
title('Total Curtailment')

figure
plot(sum(Pbat_hist,1))
title('Total Battery Power')

figure
plot(mean(SOC_hist,1))
title('Average SOC')



function [load, solar] = synthetic_profiles(Np,T)

t = (1:T);

load = zeros(Np,T);
solar = zeros(Np,T);

for i=1:Np

    base = 2 + 0.5*rand;

    % load: morning + evening peaks
    load(i,:) = base + 0.8*sin(2*pi*(t-20)/96).^2 + 0.2*randn(1,T);

    % solar: bell shape
    s = max(0,sin(pi*(t-24)/48));
    solar(i,:) = (3+rand)*s + 0.1*randn(1,T);
end
end



function F = forecast_profile(real_data,t,H)

[Np,T] = size(real_data);

F = zeros(Np,H);

for k=1:H
    idx = min(t+k-1,T);
    F(:,k) = real_data(:,idx) + 0.05*randn(Np,1);
end
end



function [lambda_imp, lambda_exp] = upper_layer_tariff(loadF,solarF,S,V0,Vmax)

[Np,H] = size(loadF);

lambda_imp = 0.25*ones(Np,H);
lambda_exp = 0.15*ones(Np,H);

for k=1:H
    Pinj = solarF(:,k)-loadF(:,k);
    Vest = V0 + S*Pinj;

    for i=1:Np
        if Vest(i) > Vmax
            lambda_exp(i,k) = 0.05;  % penalize export
        else
            lambda_exp(i,k) = 0.20;
        end
    end
end
end





function sol = lower_layer_mpc(loadF,solarF,lambda_imp,lambda_exp,...
                               SOC0,Ts,eta,Pbat_max,SOCmin,SOCmax,...
                               Curt_cost,Deg_cost,S,V0,Vmax)

[Np,H] = size(loadF);

yalmip('clear')

Pimp  = sdpvar(Np,H);
Pexp  = sdpvar(Np,H);
Pcurt = sdpvar(Np,H);
Pbat  = sdpvar(Np,H);
SOC   = sdpvar(Np,H+1);

con = [SOC(:,1)==SOC0];
obj = 0;

for k=1:H
    for i=1:Np

        con = [con, ...
            loadF(i,k) == solarF(i,k) - Pcurt(i,k) ...
            + Pimp(i,k) - Pexp(i,k) + Pbat(i,k)];

        con = [con, Pimp(i,k)>=0, Pexp(i,k)>=0, Pcurt(i,k)>=0];
        con = [con, -Pbat_max <= Pbat(i,k) <= Pbat_max];

        con = [con, SOC(i,k+1)==SOC(i,k)+Ts*Pbat(i,k)];
        con = [con, SOCmin<=SOC(i,k+1)<=SOCmax];

        V = V0 + S*(Pexp(i,k)-Pimp(i,k));
        con = [con, V<=Vmax];

        obj = obj + ...
              lambda_imp(i,k)*Pimp(i,k) ...
            - lambda_exp(i,k)*Pexp(i,k) ...
            + Curt_cost*Pcurt(i,k) ...
            + Deg_cost*abs(Pbat(i,k));
    end
end

% ops = sdpsettings('solver','gurobi','verbose',0);
ops = sdpsettings('solver','quadprog','verbose',0);
optimize(con,obj,ops)

sol.Pimp  = value(Pimp);
sol.Pexp  = value(Pexp);
sol.Pcurt = value(Pcurt);
sol.Pbat  = value(Pbat);
sol.SOC   = value(SOC);

end