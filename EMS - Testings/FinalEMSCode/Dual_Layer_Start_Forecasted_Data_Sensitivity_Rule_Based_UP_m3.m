%% =========================================================
% TWO-LAYER CONSTANT-HORIZON MPC ENERGY MANAGEMENT SYSTEM
% 48h simulation, 24h rolling optimization horizon
% UPPER LAYER: tariff optimization
% LOWER LAYER: pole-wise MPC with battery + curtailment

%% Code 2
% PCC volatge added
% soc equation done, no need to add separately as random mimic values
% real data added for simulation
% Vpcc added
% TOU added
% horizon price change added

% anith upper layer eka damma

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

data = readtable('afterforecasting_03_12.xlsx'); %forecatsed values for next 48h (96 time steps)

time_excel = data{:,1};   % First column = timestamp
timestamps = data{:,1};

Load = data{:,2:15}';     % 14 poles load (transpose!)
PV   = data{:,16:29}';

P_load =Load';
P_solar = PV';  % 14 poles solar (transpose!)

Load_tot = sum(Load);
PV_tot   = sum(PV);


% real data

data1 = readtable('afterforecasting_03_12_real.xlsx'); %forecatsed values for next 48h (96 time steps)

time_excel = data1{:,1};   % First column = timestamp
timestamps = data1{:,1};

Load_real = data1{:,2:15}';     % 14 poles load (transpose!)
PV_real   = data1{:,16:29}';

P_load_real =Load_real';
P_solar_real = PV_real';  % 14 poles solar (transpose!)

Load_tot_real = sum(Load_real);
PV_tot_real   = sum(PV_real);


% forecast error simulation

% % Define forecast error
% error_PV  = 0.05;  % 5% error
% error_Load = 0.05; % 5% error

% % Create real data from forecast
% PV_real   = PV .* (1 + error_PV * randn(size(PV)));
% Load_real = Load .* (1 + error_Load * randn(size(Load)));


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
%% READ PCC VOLTAGE FROM EXCEL
%% =========================================================

data = readtable('PCC_voltage.xlsx');

time_excel = data{:,1};      % timestamps (optional)
Vpcc = data{:,2};            % PCC voltage (pu)

Vpcc = Vpcc(:)';             % make row vector (1 x 192)



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
S=S*1; % Adjust sensitivity factor as needed

%% =========================================================
%% PER-UNIT CONVERSION
%% =========================================================
V_base = 400;           % V (LL)
S_base = 100e3;         % 100 kVA base

Z_base = V_base^2 / S_base;
S_pu   = S / Z_base;

P_net = P_load - P_solar;            % kW
P_pu  = P_net / (S_base/1000);       % kW → pu

% %% =========================================================
% %% VOLTAGE CALCULATION (SENSITIVITY METHOD)
% %% =========================================================
% V_pu = zeros(size(P_pu));

% for t = 1:size(P_pu,1)
%     V_pu(t,:) = 1 - S_pu * P_pu(t,:)';
% end

% V_poles = V_pu';     % make poles x time

%% =========================================================
%% VOLTAGE CALCULATION WITH PCC VOLTAGE
%% =========================================================

V_pu = zeros(size(P_pu));

for t = 1:size(P_pu,1)
    
    % voltage drop along feeder
    V_drop = S_pu * P_pu(t,:)';
    
    % pole voltages
    V_pu(t,:) = Vpcc(t) - V_drop';
    
end

V_poles = V_pu';   % poles x time

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
Ebat = 150; % MPC fails for 100
Pbat_max = 50;
SOCmin = 0.2*Ebat;
SOCmax = 0.8*Ebat;
SOC0   = 0.5*Ebat;
nch = 0.95;   % charging efficiency
ndis = 1.05; %1/0.95;  % discharging efficiency

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
cmin = 14;                     % min export tariff
cmax = 26;                     % max export tariff

%% =========================================================
%% TOU IMPORT TARIFF (RULE-BASED)
%% =========================================================
Timp_day = zeros(1,Nday);

for t = 1:Nday
    
    hour = (t-1)*0.25;   % convert step to hour
    
    if hour >= 18 && hour < 22
        Timp_day(t) = 40;    % peak
    elseif hour >= 6 && hour < 18
        Timp_day(t) = 30;    % day
    else
        Timp_day(t) = 20;    % night
    end
    
end

Timp = repmat(Timp_day,1,2);

% Simulate voltages at each pole (random for demo)
% V_poles = 0.95 + 0.1*rand(Npoles,Nsim);

%% =========================================================
%% RESULT STORAGE
%% =========================================================

% System-level storage
Pimp=zeros(Nday,1);
Pexp=zeros(Nday,1);
Pdis=zeros(Nday,1);
Pch=zeros(Nday,1);
Pbat=zeros(Nday,1);
Curt=zeros(Nday,1);
SOC =zeros(Nday,1);

% Pole-wise storage for export, curtailment, and tariffs
Pexp_pole = zeros(Npoles,Nday);
Curt_pole = zeros(Npoles,Nday);
Texp_pole = zeros(Npoles,Nday);

SOC_now = SOC0;
total_cost = 0;

options = optimoptions('linprog','Display','none');

%% =========================================================
%% TRUE CONSTANT HORIZON MPC WITH UPPER LAYER + POLE-WISE
%% =========================================================
for k=1:Nday
    
    Nh = Nday;               % CONSTANT horizon
    idx_start = k;
    idx_end   = k+Nh-1;
    
    %% =============================
    %% UPPER LAYER: LP TARIFF
    %% =============================
    
    % T_imp = Timp_base;
    % T_imp = Timp(k);  % update import tariff for current time step TOU, but entire horizon see one price
    % T_exp = Texp_base;
    
    % T_exp_vec = zeros(Npoles,1);           % it gets same export tariff for entire horizon
    
    % for p = 1:Npoles
    
    %     Vn = V_poles(p,k);   % voltage at pole p, time k
    
    %     if Vn <= 0.95
    %         T_exp_vec(p) = 26;
    
    %     elseif Vn >= 1.05
    %         T_exp_vec(p) = 14;
    
    %     else
    %         T_exp_vec(p) = 26 - ((Vn - 0.95)/(1.05 - 0.95))*(26 - 14);
    %     end
    
    % end
    
    %% =========================================================
    %% PRECOMPUTE EXPORT TARIFF FOR ALL POLES AND TIMES
    %% =========================================================
    
    % Texp_pole_day = zeros(Npoles, Nday);
    
    % for t = 1:Nday
    
    %     for p = 1:Npoles
    
    %         Vn = V_poles(p,t);
    
    %         if Vn <= 0.95
    %             Texp_pole_day(p,t) = 26;
    
    %         elseif Vn >= 1.05
    %             Texp_pole_day(p,t) = 14;
    
    %         else
    %             Texp_pole_day(p,t) = 26 - ((Vn - 0.95)/(1.05 - 0.95))*(26 - 14);
    %         end
    
    %     end
    
    % end
    
    % Texp_pole=repmat(Texp_pole_day,1,2); % repeat for 48h, but we will only use the relevant part in each iteration
    
    
    Texp_pole_day = zeros(Npoles,Nday);
    X_export      = zeros(Npoles,Nday);
    % options = optimoptions('linprog','Display','none');
    for t = 1:Nday
        
        % PVt = P_solar(:,t)';      % PV available
        PVt = P_solar(t,1:Npoles)';
        % V0  = Vpcc(:,t);        % base voltage
        V0 = Vpcc(t)*ones(Npoles,1);
        % V0 = V0(:);
        
        % LP objective
        
        f = -ones(Npoles,1);      % maximize export
        
        % Voltage constraints
        
        A = [ S;
            -S];
        
        b = [1.05 - V0;
            V0 - 0.95];
        
        % Bounds
        
        lb = zeros(Npoles,1);
        ub = PVt;
        
        % Solve LP
        % y = linprog(f,A,b,[],[],lb,ub);
        [y,~,exitflag] = linprog(f,A,b,[],[],lb,ub,options);
        
        if exitflag ~= 1
            y = ones(Npoles,1);   % no export allowed
        end
        X_export(:,t) = y;
        
        % Tariff calculation
        
        tariff = cmin + (cmax - cmin).*(y./max(PVt,1e-3));  % avoid division by zero
        
        Texp_pole_day(:,t) = tariff;
        
    end
    
    Texp_pole = repmat(Texp_pole_day,1,2);
    
    
    
    
    %% =============================
    %% LOWER LAYER MPC (pole-wise)
    %% =============================
    nv_per = 1 + Npoles + 2 + Npoles + 1; % [Pimp, Pexp_1..Npoles, Pch,Pdis,Curt_1..Npoles,SOC]
    nvar = Nh *nv_per; % 3072
    
    PV_h = zeros(Npoles, 192);
    Load_h = zeros(Npoles, 192);
    
    PV_h(:,1) = PV_real(:,k);
    Load_h(:,1) = Load_real(:,k);
    
    PV_h(:,2:Nh) = PV(:,k+1 : k+Nh-1);
    Load_h(:,2:Nh) = Load(:,k+1 : k+Nh-1);
    
    %% COST VECTOR
    f_mpc=zeros(nvar,1);
    for i=1:Nh
        
        
        
        idx=(i-1)*nv_per;
        % f_mpc(idx+1)=T_imp*dt;                    % import
        % f_mpc(idx+2:idx+1+Npoles)=-dt*T_exp_vec;  % export revenue pole wise
        f_mpc(idx+2+Npoles)=c_deg*dt;             % Pch degradation
        f_mpc(idx+3+Npoles)=c_deg*dt;             % Pdis degradation
        f_mpc(idx+4+Npoles:idx+3+2*Npoles)=c_curt*dt; % Curtailment pole wise , c_curt - scaler value, not matrix
    end
    
    for j = 1:Nh
        
        jdx = (j-1)*nv_per;
        
        T_imp_j = Timp(k+j-1);
        %T_imp_j = Timp(min(k+j-1,Nday));
        f_mpc(jdx+1) = T_imp_j * dt;
        
        T_exp_j = Texp_pole(:,k+j-1);
        
        f_mpc(idx+2:idx+1+Npoles) = -dt*T_exp_j;
        
    end
    
    
    %% BOUNDS
    lb=zeros(nvar,1); ub=inf(nvar,1);
    for i=1:Nh
        idx=(i-1)*nv_per;
        % Pimp max limit
        ub(idx+1)=Pgrid_max;
        % Pexp
        ub(idx+2:idx+1+Npoles)=PV_h(:,idx_start+i-1); % pole pv e time step eke max eka
        %lb(idx+2:idx+1+Npoles)=max(0,0.5*PV(:,idx_start+i-1)-Load(:,idx_start+i-1)); % pole load e time step eke min eka (negative export)
        % Pch,Pdis
        ub(idx+2+Npoles)=Pbat_max;
        ub(idx+3+Npoles)=Pbat_max;
        % Curtailment
        ub(idx+4+Npoles:idx+3+2*Npoles)=1*PV_h(:,idx_start+i-1);   % PV should be Npoles × time       % can limit curtailment from here - fairness constraint
        % lb(idx+4+Npoles:idx+3+2*Npoles)=0.1*PV(:,idx_start+i-1);                     % no negative curtailment
        
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
        row(idx+2+Npoles)=-1;  % Pch
        row(idx+3+Npoles)=1; % Pdis
        row(idx+4+Npoles:idx+3+2*Npoles)=-1; % Curtailment
        beq_i = sum(Load_h(:,idx_start+i-1)) - sum(PV_h(:,idx_start+i-1));
        Aeq=[Aeq;row]; beq=[beq; beq_i]; % just add Aeq to previous Aeq matrix, as additional row, same for beq
    end
    
    % SOC dynamics
    for i=1:Nh-1
        row=zeros(1,nvar);
        idx=(i-1)*nv_per; idx2=i*nv_per;
        row(idx+4+2*Npoles)=1;          % SOC(h)
        row(idx+2+Npoles)=nch*dt;          % Pch
        row(idx+3+Npoles)=-ndis*dt;          % Pdis
        row(idx2+4+2*Npoles)=-1;        % SOC(h+1)
        Aeq=[Aeq;row]; beq=[beq;0];
    end
    
    % Initial SOC measured
    row=zeros(1,nvar);
    row(4+2*Npoles)=1;
    Aeq=[Aeq;row]; beq=[beq;SOC_now];
    
    %% INEQUALITY: Grid limit
    A=[]; b=[];
    
    % PV allocation constraint: Pexp_p + Curt_p <= PV_p
    for i=1:Nh
        idx=(i-1)*nv_per;
        row1=zeros(1,nvar); row1(idx+1)=1;;
        row2=zeros(1,nvar); row2(idx+2:idx+1+Npoles)=1;
        A=[A;row1;row2];
        b=[b;Pgrid_max;Pgrid_max/2];
    end
    
    for i = 1:Nh
        
        idx = (i-1)*nv_per;
        
        for p = 1:Npoles
            
            row = zeros(1,nvar);
            
            % Export variable
            row(idx + 1 + p) = 1;
            
            % Curtailment variable
            row(idx + 3 + Npoles + p) = 1;
            
            % Add constraint
            A = [A; row];
            b = [b; PV_h(p, idx_start + i - 1)];
            
        end
        
    end
    
    
    %% SOLVE MPC
    [x,fval,exitflag,output] = linprog(f_mpc,A,b,Aeq,beq,lb,ub,options);
    
    if isempty(x)
        warning('Optimization failed at step %d', k);
        disp(exitflag);
        disp(output.message);
        break
    end
    x = reshape(x,nv_per,Nh)';   %Each timestep is one row → easy to send first step to actuators, then recede horizon.
    
    %% APPLY FIRST STEP
    
    Pimp(k)=x(1,1);
    
    % POLE EXPORTS
    Pexp_pole(:,k)=x(1,2:1+Npoles)';        % store export for each pole
    Pexp(k)=sum(Pexp_pole(:,k));            % store total export
    
    % CURTAILMENT
    Curt_pole(:,k)=x(1,4+Npoles:3+2*Npoles)';
    Curt(k)=sum(Curt_pole(:,k));
    
    % BATTERY
    Pch(k)=x(1,2+Npoles);
    Pdis(k)=x(1,3+Npoles);
    
    Pbat_k = x(1,2+Npoles)-x(1,3+Npoles);  % Pbat = Pch - Pdis
    Pbat(k)=Pbat_k;
    
    SOC_now = SOC_now + dt*(nch*x(1,2+Npoles)-ndis*x(1,3+Npoles));
    SOC(k)=SOC_now;
    
    % add real SOC
    % SOC_now = SOC_now + dt*(nch*x(1,2+Npoles)-ndis*x(1,3+Npoles));
    % SOC(k)=SOC_now;
    
    
    % TARIFF
    % Texp_pole(:,k)=T_exp_vec;
    
    total_cost = total_cost + f_mpc(1:nv_per)'*x(1,:)';
    
end

SOC_pct = 100*SOC/Ebat;
disp(['Total daily cost (TRUE MPC) = ',num2str(total_cost)])

%% =========================================================
%% PLOTS
%% =========================================================
figure
time_plot = 1:Nday;   % 1 to 96 time steps for 24h horizon

subplot(5,1,1)
plot(time_plot, Load_tot(1:Nday), 'k', time_plot, PV_tot(1:Nday), 'g','LineWidth',1.4)
xlabel('Time Step (15 min)')
ylabel('kW')
xlim([1 100])
grid on
legend('Load','PV')
title('Load & PV')

subplot(5,1,2)
plot(Pimp,'b','LineWidth',1.4); hold on; plot(Pexp,'r','LineWidth',1.4)
yline(Pgrid_max,'--k')
legend('Import','Export','Grid Limit')
title('Grid Power'); grid on

subplot(5,1,3)
plot(Pch,'b','LineWidth',1.4);hold on; plot(Pdis,'LineWidth',1.4);
yline(0)
legend('Charge','Discharge')
title('Battery Power');
grid on

subplot(5,1,4)
plot(SOC_pct,'LineWidth',1.4); ylim([0 100])
title('SOC (%)'); grid on

subplot(5,1,5)
plot(Curt,"LineWidth",1.4); yline(0)
title('Curtailment'); grid on
xlabel('Step')


%% =========================================================
%% POLE-WISE NET INJECTION CALCULATION
%% =========================================================

Pnet_pole = zeros(Npoles,Nday);

for k = 1:Nday
    for p = 1:Npoles
        % Optimized net injection
        Pnet_pole(p,k) = PV(p,k) - Load(p,k) - Curt_pole(p,k);     % Pole export -(+)
        % This should equal Pexp_pole(p,k)
    end
end

%% =========================================================
%% POLE-WISE SCHEDULED SOLAR INJECTION CALCULATION
%% =========================================================

Psche_soalr_pole = zeros(Npoles,Nday);

for k = 1:Nday
    for p = 1:Npoles
        Psche_soalr_pole(p,k) = PV(p,k) - Curt_pole(p,k);
    end
end

%% =========================================================
%% OUTPUTS TO EXCEL
%% =========================================================

filename = 'MPC_Results_Rule-Based_PoleWise.xlsx';

% --- System level ---
SystemTable = table;

SystemTable.Time = time_excel(1:Nday);
SystemTable.Load_total  = Load_tot(1:Nday)';
SystemTable.Solar_total = PV_tot(1:Nday)';
SystemTable.Curtailment = Curt;
SystemTable.P_import    = Pimp;
SystemTable.P_export    = Pexp;
SystemTable.P_bat_ch    = Pch;
SystemTable.P_bat_dis   = Pdis;
SystemTable.P_battery   = Pbat;
SystemTable.SOC_percent = SOC_pct;

writetable(SystemTable,filename,'Sheet','System');

% --- Pole-wise Net Injection ---
PoleNetTable = array2table(Pnet_pole');
PoleNetTable.Time = time_excel(1:Nday);

% Move Time column to the first position
PoleNetTable = movevars(PoleNetTable,'Time','Before',1);

% Rename columns nicely
for p = 1:Npoles
    PoleNetTable.Properties.VariableNames{p+1} = ...
        ['Pole_' num2str(p) '_Injection_kW'];
end

writetable(PoleNetTable,filename,'Sheet','Pole_Net_Injection');

% For Power Flow Analysis
PowerFlowTable = table;
PowerFlowTable.Time = time_excel(1:Nday);
for p = 1:Npoles
    PowerFlowTable.(['Pole_' num2str(p) '_Load_kW']) = Load(p,1:Nday)';
end
for p = 1:Npoles
    PowerFlowTable.(['Pole_' num2str(p) '_Psche_Solar_kW']) = Psche_soalr_pole(p,1:Nday)';
end
PowerFlowTable.P_import = Pimp;
PowerFlowTable.P_export = Pexp;
PowerFlowTable.P_battery = Pbat;

writetable(PowerFlowTable,filename,'Sheet','Pole_Scheduled_Solar');


% For Pole-wise export and curtailment
PoleExportCurtailmentTable = table;

PoleExportCurtailmentTable.Time = time_excel(1:Nday);
for p = 1:Npoles
    PoleExportCurtailmentTable.(['Pole_' num2str(p) '_Export_kW']) = Pexp_pole(p,1:Nday)';
    PoleExportCurtailmentTable.(['Pole_' num2str(p) '_Curtailment_kW']) = Curt_pole(p,1:Nday)';
end

writetable(PoleExportCurtailmentTable,filename,'Sheet','Pole_Export_Curtailment');

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
%% VOLTAGE vs TARIFF (comparison)
%% =========================================================
figure('Name','Voltage vs Tariff (Pole 1 example)')

yyaxis left
plot(time_plot,V_poles(1,1:Nday),'LineWidth',1.4)
ylabel('Voltage (p.u.)')

yyaxis right
plot(time_plot,Texp_pole_day(1,:),'LineWidth',1.4)
ylabel('Export Tariff (LKR/kWh)')

xlabel('Hour')
title('Voltage–Tariff Relationship (Pole 1)')
grid on


%% =========================================================
%% POLE-WISE VOLTAGE PROFILES - SENSITIVITY ANALYSIS
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

