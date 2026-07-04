import pandapower as pp
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


# PARAMETERS

V_HV_kV = 11
V_LV_kV = 0.4
S_tr_MVA = 0.25          # 250 kVA transformer

r_per_km = 0.443
x_per_km = 0.08          # realistic LV cable reactance 

dist_m = [23.35,20.42,20.09,21.74,23.76,24.06,22,21.4,
          32.07,24.19,13.57,21.79,28.15,19.78]

dist_km = np.array(dist_m)/1000
r_per_km = r_per_km


# READ EXCEL LOAD AND SOLAR DATA

data = pd.read_excel(
    r"2 TOU_Fixed_Fixed/MPC_Results_TOU_Fixed_Fixed.xlsx",
    sheet_name="Pole_Scheduled_Solar"
)

timestamps = data.iloc[:,0]

P_load = data.iloc[:,1:15].values   # kW
P_solar = data.iloc[:,15:29].values # kW
P_bat = data.iloc[:,31].values   # Battery power column (kW)

n_time = P_load.shape[0]
n_bus = 14


# READ PCC VOLTAGE PROFILE

pcc_data = pd.read_excel("PCC_voltage.xlsx")

Vpcc = pcc_data.iloc[:,1].values   # column with pu voltage

# CREATE NETWORK

net = pp.create_empty_network()


# Create HV and LV buses

bus_hv = pp.create_bus(net, vn_kv=V_HV_kV)
bus_lv = pp.create_bus(net, vn_kv=V_LV_kV)

# External grid at HV side
pp.create_ext_grid(net, bus=bus_hv, vm_pu=1.0)


# Transformer 11/0.4 kV

pp.create_transformer_from_parameters(
    net,
    hv_bus=bus_hv,
    lv_bus=bus_lv,
    sn_mva=S_tr_MVA,
    vn_hv_kv=V_HV_kV,
    vn_lv_kv=V_LV_kV,
    vk_percent=4,        # typical 4% impedance
    vkr_percent=1.2,     # typical copper loss
    pfe_kw=0,
    i0_percent=0
)


# Create feeder buses (14 poles)

buses = [bus_lv]

for i in range(n_bus):
    buses.append(pp.create_bus(net, vn_kv=V_LV_kV))


# Create feeder lines

for i in range(n_bus):
    pp.create_line_from_parameters(
        net,
        from_bus=buses[i],
        to_bus=buses[i+1],
        length_km=dist_km[i],
        r_ohm_per_km=r_per_km,
        x_ohm_per_km=x_per_km,
        c_nf_per_km=0,
        max_i_ka=0.4
    )

# CREATE BATTERY AT PCC (LV BUS)

battery = pp.create_storage(
    net,
    bus=bus_lv,
    p_mw=0,
    max_e_mwh=0.1,   # optional energy capacity
    soc_percent=50,  # initial SOC
    min_e_mwh=0,
    max_p_mw=0.05,
    min_p_mw=-0.05
)

# Create loads and solar

loads = []
sgens = []
line_loading = []
trafo_loading = []
loss_series = []
line_p_from = []
line_p_to = []

# transformer power flow
trafo_power = net.res_trafo.p_hv_mw.values

# store during simulation
trafo_power_series = []

for i in range(n_bus):
    loads.append(
        pp.create_load(net, bus=buses[i+1], p_mw=0, q_mvar=0)
    )
    sgens.append(
        pp.create_sgen(net, bus=buses[i+1], p_mw=0, q_mvar=0)
    )


# TIME SERIES POWER FLOW

V_results = np.zeros((n_time, n_bus))
P_grid_results = np.zeros(n_time)

for t in range(n_time):
    
    # UPDATE PCC VOLTAGE
    # net.ext_grid.at[0, "vm_pu"] = Vpcc[t]

    for i in range(n_bus):

        # Load (kW → MW)
        net.load.at[loads[i], "p_mw"] = P_load[t,i] / 1000
        net.load.at[loads[i], "q_mvar"] = 0   # change if PF known

        # Solar (kW → MW)
        net.sgen.at[sgens[i], "p_mw"] = P_solar[t,i] / 1000
        net.sgen.at[sgens[i], "q_mvar"] = 0

        # Battery power (kW → MW)
        net.storage.at[battery, "p_mw"] = -P_bat[t] / 1000

    # Run full AC Newton-Raphson
    pp.runpp(net)

    # Store LV feeder voltages only (skip HV + LV bus)
    V_results[t,:] = net.res_bus.vm_pu.values[2:]
    P_grid_results[t] = net.res_ext_grid.p_mw.values[0]
    line_loading.append(net.res_line.loading_percent.values)
    trafo_loading.append(net.res_trafo.loading_percent.values)
    trafo_power_series.append(net.res_trafo.p_hv_mw.values[0])
    loss = net.res_line.pl_mw.sum()
    loss_series.append(loss)
    line_p_from.append(net.res_line.p_from_mw.values)
    line_p_to.append(net.res_line.p_to_mw.values)


line_loading = np.array(line_loading)
trafo_loading = np.array(trafo_loading)
rafo_power_series = np.array(trafo_power_series)
loss_series = np.array(loss_series)
line_p_from = np.array(line_p_from)
line_p_to = np.array(line_p_to)

# SAVE RESULTS

df_out = pd.DataFrame(
    V_results,
    columns=[f"Voltage_Pole_{i+1}_pu" for i in range(n_bus)]
)

df_out["Grid_Power_MW"] = P_grid_results

df_out.insert(0,"Timestamp",timestamps)

df_out.to_excel("2 TOU_Fixed_Fixed/voltage_output_panda_powerflow_TOU_Fixed_Fixed.xlsx", index=False)

print("Power flow completed successfully.")


import matplotlib.pyplot as plt


# PLOT 1: All pole voltages vs time

plt.figure(figsize=(12,6))

for i in range(n_bus):
    plt.plot(V_results[:, i], linewidth=1, label=f'Pole {i+1}')

# Voltage limits
plt.axhline(0.95, linestyle='--', linewidth=2, label='Low voltage limit (0.95 pu)')
plt.axhline(1.05, linestyle='--', linewidth=2, label='High voltage limit (1.05 pu)')

plt.xlabel('Time Step')
plt.ylabel('Voltage (pu)')
plt.title('Voltage Profile of 14 Poles')
plt.grid(True)
plt.legend(ncol=2, fontsize=8)
plt.tight_layout()
plt.show()

# PLOT 2: Voltage envelope (min/max across poles)

V_min = V_results[:,2]
V_max = V_results[:,13]

plt.figure(figsize=(12,6))
plt.plot(V_min, label='Minimum voltage')
plt.plot(V_max, label='Maximum voltage')

plt.axhline(0.95, linestyle='--', linewidth=2, label='Low voltage limit (0.95 pu)')
plt.axhline(1.05, linestyle='--', linewidth=2, label='High voltage limit (1.05 pu)')

plt.xlabel('Time Step')
plt.ylabel('Voltage (pu)')
plt.title('Feeder Voltage Envelope (Min–Max across 14 Poles)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()





# ===============================
# VALIDATION RESULTS
# ===============================
print("----- Voltage Limits -----")
print("Max voltage:", V_results.max())
print("Min voltage:", V_results.min())

print("----- Line Loading -----")
print("Max line loading:", line_loading.max())

print("----- Transformer Loading -----")
print("Max transformer loading:", trafo_loading.max())

# detect violations
voltage_violation = np.logical_or(V_results > 1.05, V_results < 0.95)

if voltage_violation.any():
    print("Voltage violations detected")
else:
    print("No voltage violations")

if line_loading.max() > 100:
    print("Line overload detected")

if trafo_loading.max() > 100:
    print("Transformer overload detected")


# ===============================
# GRAPHICAL OUTPUTS
# ===============================

plt.figure()
plt.imshow(V_results.T, aspect='auto')
plt.xlabel("Time Step")
plt.ylabel("Bus Number")
plt.title("Voltage Variation Across Feeder")
plt.colorbar(label="Voltage (pu)")
plt.show()

plt.figure()
plt.plot(trafo_loading)
plt.xlabel("Time Step")
plt.ylabel("Loading (%)")
plt.title("Transformer Loading Variation")
plt.grid()
plt.show()

plt.figure()
for i in range(line_loading.shape[1]):
    plt.plot(line_loading[:,i])
plt.xlabel("Time Step")
plt.ylabel("Line Loading (%)")
plt.title("Line Loading Variation")
plt.show()


plt.figure()
plt.plot(trafo_power_series)
plt.axhline(0, linestyle="--")
plt.xlabel("Time Step")
plt.ylabel("Transformer Power (MW)")
plt.title("Grid Import / Export Through Transformer")
plt.grid(True)
plt.show()


plt.figure()
plt.plot(loss_series*1000)
plt.xlabel("Time Step")
plt.ylabel("Feeder Loss (kW)")
plt.title("Distribution Feeder Losses")
plt.grid(True)
plt.show()


plt.figure()
for i in range(line_p_from.shape[1]):
    plt.plot(line_p_from[:, i], label=f'Line {i+1}')

plt.axhline(0, linestyle='--')
plt.xlabel("Time Step")
plt.ylabel("Power Flow (MW)")
plt.title("Line Power Flow Direction")
plt.legend(ncol=2, fontsize=8)
plt.grid()
plt.show()


plt.figure()
plt.imshow(line_p_from.T, aspect='auto')
plt.colorbar(label="MW")
plt.xlabel("Time Step")
plt.ylabel("Line Number")
plt.title("Power Flow Direction Heatmap")
plt.show()

t = 80  # example

plt.figure()
plt.plot(line_p_from[t, :], marker='o')
plt.axhline(0, linestyle='--')
plt.xlabel("Line Number (1 = near transformer)")
plt.ylabel("Power Flow (MW)")
plt.title(f"Power Flow Along Feeder at Time {t}")
plt.grid()
plt.show()