import opendssdirect as dss
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# =========================
# LOAD EMS RESULTS
# =========================
data = pd.read_excel("MPC_Results_Rule-Based_PoleWise.xlsx")

# =========================
# LOAD OPENDSS MODEL
# =========================
dss.Command(r'Compile "E:\OneDrive\Bsc.Eng - Final\SEM 07 - OD\FYP\FYP 2\PR 1\EMS - VS Code\simple_model\PR2\FinalEMSCode\feeder.dss"')

T = len(data)
voltages = []

# =========================
# TIME SERIES SIMULATION
# =========================
for t in range(T):

    # update loads and PV
    for i in range(1,15):

        load = data[f"Pole_{i}_Load_kW"][t]
        pv = data[f"Pole_{i}_Psche_Solar_kW"][t]

        dss.Command(f"Edit Load.Load{i} kw={load}")
        dss.Command(f"Edit PVSystem.PV{i} pmpp={pv}")

    # battery power
    bat = data["P_battery"][t]
    dss.Command(f"Edit Storage.BESS kw={bat}")

    # solve power flow
    dss.Command("Solve")

    # store bus voltages
    v = dss.Circuit.AllBusMagPu()
    voltages.append(v)

voltages = np.array(voltages)

print(voltages)
# =========================
# VOLTAGE PROFILE PLOT
# =========================
plt.figure()

for t in range(0,96,10):
    plt.plot(voltages[t], label=f"t={t}")

plt.xlabel("Bus")
plt.ylabel("Voltage (pu)")
plt.title("Voltage Profile Along Feeder")
plt.legend()
plt.grid()

plt.show()