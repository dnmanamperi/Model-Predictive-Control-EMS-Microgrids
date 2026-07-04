# import pandapower as pp
# import pandas as pd
# import numpy as np

# # ===============================
# # PARAMETERS
# # ===============================

# V_base_kV = 0.4        # base voltage (change if needed)
# r_per_km = 0.443

# dist_m = [23.35,20.42,20.09,21.74,23.76,24.06,22,21.4,
#           32.07,24.19,13.57,21.79,28.15,19.78]

# dist_km = np.array(dist_m)/1000
# R = r_per_km * dist_km
# X = 0.00001 * dist_km   # assume example reactance (adjust if known)

# # ===============================
# # READ EXCEL
# # ===============================

# data = pd.read_excel(r"simple_model\PR2\voltageSensitivity\example_14_pole_load_solar_real.xlsx")

# timestamps = data.iloc[:,0]

# P_load = data.iloc[:,1:15].values   # 14 loads
# P_solar = data.iloc[:,15:29].values # 14 solar

# P_net = P_load - P_solar  # kW

# n_time = P_net.shape[0]
# n_bus = 14

# # ===============================
# # CREATE NETWORK
# # ===============================

# net = pp.create_empty_network()




# # Create buses
# buses = []
# for i in range(n_bus+1):  # include slack bus
#     buses.append(pp.create_bus(net, vn_kv=V_base_kV))

# # Slack bus at bus 0
# pp.create_ext_grid(net, bus=buses[0], vm_pu=1.0)

# # Create lines (radial chain)
# for i in range(n_bus):
#     pp.create_line_from_parameters(
#         net,
#         from_bus=buses[i],
#         to_bus=buses[i+1],
#         length_km=dist_km[i],
#         r_ohm_per_km=r_per_km,
#         x_ohm_per_km=0.0000001,   # adjust if you know real X
#         c_nf_per_km=0,
#         max_i_ka=1
#     )

# # Create load elements (one per pole)
# loads = []
# for i in range(n_bus):
#     loads.append(pp.create_load(net, bus=buses[i+1], p_mw=0, q_mvar=0))

# # Create solar generators (as sgen)
# sgens = []
# for i in range(n_bus):
#     sgens.append(pp.create_sgen(net, bus=buses[i+1], p_mw=0, q_mvar=0))

# # ===============================
# # TIME SERIES POWER FLOW
# # ===============================

# V_results = np.zeros((n_time, n_bus))

# for t in range(n_time):

#     # Update loads (convert kW → MW)
#     for i in range(n_bus):
#         net.load.at[loads[i], "p_mw"] = P_load[t,i] / 1000
#         net.load.at[loads[i], "q_mvar"] = 0  # assume unity PF

#         net.sgen.at[sgens[i], "p_mw"] = P_solar[t,i] / 1000
#         net.sgen.at[sgens[i], "q_mvar"] = 0

#     # Run AC power flow
#     pp.runpp(net)

#     # Store voltages (skip slack)
#     V_results[t,:] = net.res_bus.vm_pu.values[1:]

# # ===============================
# # SAVE RESULTS
# # ===============================

# df_out = pd.DataFrame(V_results,
#                       columns=[f"Voltage_Pole_{i+1}_pu" for i in range(n_bus)])

# df_out.insert(0,"Timestamp",timestamps)

# df_out.to_excel("pandapower_voltage_results.xlsx", index=False)

# print("Power flow completed and saved.")












# with transformer

import pandapower as pp
import pandas as pd
import numpy as np

# ===============================
# PARAMETERS
# ===============================

V_HV_kV = 11
V_LV_kV = 0.4
S_tr_MVA = 0.25          # 250 kVA transformer

r_per_km = 0.443*2.5
x_per_km = 0.0001          # realistic LV cable reactance (adjust if known)

dist_m = [23.35,20.42,20.09,21.74,23.76,24.06,22,21.4,
          32.07,24.19,13.57,21.79,28.15,19.78]

dist_km = np.array(dist_m)/1000

# ===============================
# READ EXCEL
# ===============================

data = pd.read_excel(
    r"Reconstructed_14Pole_Load_Solar.xlsx"
)

timestamps = data.iloc[:,0]

P_load = data.iloc[:,1:15].values   # kW
P_solar = data.iloc[:,15:29].values # kW

n_time = P_load.shape[0]
n_bus = 14

# ===============================
# CREATE NETWORK
# ===============================

net = pp.create_empty_network()

# -------------------------------
# Create HV and LV buses
# -------------------------------

bus_hv = pp.create_bus(net, vn_kv=V_HV_kV)
bus_lv = pp.create_bus(net, vn_kv=V_LV_kV)

# External grid at HV side
pp.create_ext_grid(net, bus=bus_hv, vm_pu=1.0)

# -------------------------------
# Transformer 11/0.4 kV
# -------------------------------

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

# -------------------------------
# Create feeder buses (14 poles)
# -------------------------------

buses = [bus_lv]

for i in range(n_bus):
    buses.append(pp.create_bus(net, vn_kv=V_LV_kV))

# -------------------------------
# Create feeder lines
# -------------------------------

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

# -------------------------------
# Create loads and solar
# -------------------------------

loads = []
sgens = []

for i in range(n_bus):
    loads.append(
        pp.create_load(net, bus=buses[i+1], p_mw=0, q_mvar=0)
    )
    sgens.append(
        pp.create_sgen(net, bus=buses[i+1], p_mw=0, q_mvar=0)
    )

# ===============================
# TIME SERIES POWER FLOW
# ===============================

V_results = np.zeros((n_time, n_bus))

for t in range(n_time):

    for i in range(n_bus):

        # Load (kW → MW)
        net.load.at[loads[i], "p_mw"] = P_load[t,i] / 1000
        net.load.at[loads[i], "q_mvar"] = 0   # change if PF known

        # Solar (kW → MW)
        net.sgen.at[sgens[i], "p_mw"] = P_solar[t,i] / 1000
        net.sgen.at[sgens[i], "q_mvar"] = 0

    # Run full AC Newton-Raphson
    pp.runpp(net)

    # Store LV feeder voltages only (skip HV + LV bus)
    V_results[t,:] = net.res_bus.vm_pu.values[2:]

# ===============================
# SAVE RESULTS
# ===============================

df_out = pd.DataFrame(
    V_results,
    columns=[f"Voltage_Pole_{i+1}_pu" for i in range(n_bus)]
)

df_out.insert(0,"Timestamp",timestamps)

df_out.to_excel("pandapower_voltage_results.xlsx", index=False)

print("Power flow completed successfully.")