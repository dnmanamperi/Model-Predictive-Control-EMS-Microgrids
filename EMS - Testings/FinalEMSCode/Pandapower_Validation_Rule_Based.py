# import pandapower as pp
# import pandapower.plotting as plot
# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # ===============================================
# # READ YOUR EXCEL RESULTS
# # ===============================================

# file_path = r"MPC_Results_Rule-Based_PoleWise.xlsx"

# system_data = pd.read_excel(file_path, sheet_name="System")
# pole_data   = pd.read_excel(file_path, sheet_name="Pole_Net_Injection")

# time_steps = len(system_data)

# # Extract pole injection matrix
# P_poles = pole_data.iloc[:, :-1].values  # exclude time column

# # ===============================================
# # FEEDER PARAMETERS
# # ===============================================

# r_ohm_per_km = 0.443
# x_ohm_per_km = 0.08
# length_km = np.array([23.35,20.42,20.09,21.74,23.76,24.06,
#                       22,21.4,32.07,24.19,13.57,21.79,28.15,19.78]) / 1000

# # ===============================================
# # FUNCTION TO RUN POWER FLOW
# # ===============================================

# def run_case(battery_bus=None):

#     voltage_results = []

#     for t in range(time_steps):

#         net = pp.create_empty_network()

#         # Create buses
#         buses = []
#         for i in range(14):
#             buses.append(pp.create_bus(net, vn_kv=0.4))

#         # Slack bus at feeder head
#         slack = pp.create_bus(net, vn_kv=0.4)
#         pp.create_ext_grid(net, slack)

#         # Create radial lines
#         for i in range(14):
#             from_bus = slack if i == 0 else buses[i-1]
#             to_bus = buses[i]

#             pp.create_line_from_parameters(
#                 net,
#                 from_bus=from_bus,
#                 to_bus=to_bus,
#                 length_km=length_km[i],
#                 r_ohm_per_km=r_ohm_per_km,
#                 x_ohm_per_km=x_ohm_per_km,
#                 c_nf_per_km=0,
#                 max_i_ka=0.2
#             )

#         # Add pole injections as generators
#         for i in range(14):
#             p_mw = P_poles[t, i] / 1000  # kW → MW
#             pp.create_sgen(net, buses[i], p_mw=p_mw, q_mvar=0)

#         # Add battery if defined
#         if battery_bus is not None:
#             # Use battery dispatch from Excel
#             p_bat = system_data.loc[t, "P_battery"] / 1000
#             pp.create_sgen(net, buses[battery_bus], p_mw=p_bat, q_mvar=0)

#         # Run power flow
#         pp.runpp(net)

#         voltage_results.append(net.res_bus.vm_pu.values[1:15])

#     return np.array(voltage_results)


# # ===============================================
# # RUN THREE CASES
# # ===============================================

# # print("Running Case 1: No battery")
# # V_base = run_case()

# print("Running Case 2: Battery at Pole 1")
# V_bat1 = run_case(battery_bus=0)

# # print("Running Case 3: Battery at Pole 14")
# # V_bat14 = run_case(battery_bus=13)

# # ===============================================
# # PLOT VOLTAGE PROFILES
# # ===============================================

# plt.figure(figsize=(10,6))

# # plt.plot(V_base[:, -1], label="No Battery")
# plt.plot(V_bat1[:, -1], label="Battery @ Pole 1")
# # plt.plot(V_bat14[:, -1], label="Battery @ Pole 14")

# plt.axhline(1.05, linestyle="--", color="red")
# plt.axhline(0.95, linestyle="--", color="red")

# # plt.title("Voltage at Last Pole (Worst Case)")
# # plt.xlabel("Time Step")
# # plt.ylabel("Voltage (p.u.)")
# # plt.legend()
# # plt.grid(True)
# plt.show()










import pandapower as pp
import pandas as pd
import numpy as np

# ===============================
# SETTINGS
# ===============================
excel_file = "MPC_Results_Rule-Based_PoleWise.xlsx"   # your EMS result file
n_poles = 14

# transformer parameters (11 kV / 0.4 kV 250 kVA)
V_HV = 11
V_LV = 0.4
S_TR = 0.25

# line parameters (typical LV cable)
r_per_km = 0.443
x_per_km = 0.0001

# distances between poles (meters) from field data
dist_m = [23.35, 20.42, 20.09, 21.74, 23.76, 24.06, 22, 21.4,
          32.07, 24.19, 13.57, 21.79, 28.15, 19.78]

# convert to km for pandapower
line_lengths_km = [d/1000 for d in dist_m]

# ===============================
# LOAD EMS DATA
# ===============================
ems = pd.read_excel(excel_file)
T = len(ems)

# ===============================
# CREATE NETWORK
# ===============================
net = pp.create_empty_network()

# HV bus
bus_hv = pp.create_bus(net, vn_kv=V_HV, name="HV_bus")

# LV buses (poles)
buses = []
for i in range(n_poles):
    buses.append(pp.create_bus(net, vn_kv=V_LV, name=f"Pole_{i+1}"))

# external grid
pp.create_ext_grid(net, bus_hv)

# transformer
pp.create_transformer_from_parameters(
    net,
    hv_bus=bus_hv,
    lv_bus=buses[0],
    sn_mva=S_TR,
    vn_hv_kv=V_HV,
    vn_lv_kv=V_LV,
    vk_percent=4,
    vkr_percent=1,
    pfe_kw=0,
    i0_percent=0
)

# lines between poles (radial feeder)
for i in range(n_poles-1):

    length_km = line_lengths_km[i]

    pp.create_line_from_parameters(
        net,
        from_bus=buses[i],
        to_bus=buses[i+1],
        length_km=length_km,
        r_ohm_per_km=r_per_km,
        x_ohm_per_km=x_per_km,
        c_nf_per_km=0,
        max_i_ka=0.2
    )

# ===============================
# CREATE LOADS AND PV
# ===============================
loads = []
pvs = []

for i in range(n_poles):
    loads.append(pp.create_load(net, buses[i], p_mw=0, name=f"Load_{i+1}"))
    pvs.append(pp.create_sgen(net, buses[i], p_mw=0, name=f"PV_{i+1}"))

# ===============================
# BATTERY AT POLE 1
# ===============================
battery = pp.create_storage(
    net,
    bus=buses[0],
    p_mw=0,
    max_e_mwh=0.2,
    soc_percent=50,
    min_e_mwh=0,
    name="Community_BESS"
)

# ===============================
# TIME SERIES POWER FLOW
# ===============================
voltages = []
line_loading = []
trafo_loading = []
loss_series = []

# transformer power flow
trafo_power = net.res_trafo.p_hv_mw.values

# store during simulation
trafo_power_series = []

for t in range(T):

    # update loads and PV
    for i in range(n_poles):

        load_kw = ems[f"Pole_{i+1}_Load_kW"][t]
        pv_kw = ems[f"Pole_{i+1}_Psche_Solar_kW"][t]

        net.load.at[loads[i], "p_mw"] = load_kw / 1000
        net.sgen.at[pvs[i], "p_mw"] = pv_kw / 1000

    # battery power from EMS
    p_bat = ems["P_battery"][t] / 1000

    # convention: positive = discharge
    net.storage.at[battery, "p_mw"] = -p_bat

    # run power flow
    pp.runpp(net)

    voltages.append(net.res_bus.vm_pu.values)
    line_loading.append(net.res_line.loading_percent.values)
    trafo_loading.append(net.res_trafo.loading_percent.values)
    trafo_power_series.append(net.res_trafo.p_hv_mw.values[0])
    loss = net.res_line.pl_mw.sum()
    loss_series.append(loss)

voltages = np.array(voltages)
line_loading = np.array(line_loading)
trafo_loading = np.array(trafo_loading)
rafo_power_series = np.array(trafo_power_series)
loss_series = np.array(loss_series)

# ===============================
# VALIDATION RESULTS
# ===============================
print("----- Voltage Limits -----")
print("Max voltage:", voltages.max())
print("Min voltage:", voltages.min())

print("----- Line Loading -----")
print("Max line loading:", line_loading.max())

print("----- Transformer Loading -----")
print("Max transformer loading:", trafo_loading.max())

# detect violations
voltage_violation = np.logical_or(voltages > 1.05, voltages < 0.95)

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
import matplotlib.pyplot as plt

# worst bus voltage over time
worst_max_v = voltages.max(axis=1)
worst_min_v = voltages.min(axis=1)

plt.figure()
plt.plot(worst_max_v, label="Max Bus Voltage")
plt.plot(worst_min_v, label="Min Bus Voltage")
plt.axhline(1.05, linestyle="--", label="Upper Limit")
plt.axhline(0.95, linestyle="--", label="Lower Limit")
plt.xlabel("Time Step")
plt.ylabel("Voltage (pu)")
plt.title("Feeder Voltage Limits Over Time")
plt.legend()
plt.grid(True)
plt.show()

# print(voltages)

# for t in range(0,96,10):           # get to one plot
#     plt.figure()
#     plt.plot(voltages[t])
#     plt.xlabel("Bus Number")
#     plt.ylabel("Voltage (pu)")
#     plt.title(f"Voltage Profile at time {t}")
#     plt.grid()

plt.figure()

for t in range(0,96,10):
    plt.plot(voltages[t], label=f"t={t}")

plt.xlabel("Bus Number")
plt.ylabel("Voltage (pu)")
plt.title("Voltage Profiles Along Feeder at Different Times")
plt.legend()
plt.grid(True)

plt.show()

plt.figure()
plt.imshow(voltages.T, aspect='auto')
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







    

# convert to array
import numpy as np
t
# plot
import matplotlib.pyplot as plt

plt.figure()
plt.plot(trafo_power_series*1000)
plt.axhline(0, linestyle="--")
plt.xlabel("Time Step")
plt.ylabel("Transformer Power (kW)")
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