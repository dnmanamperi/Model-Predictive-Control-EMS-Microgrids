import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
# =========================================================
# 1. READ TOTAL IMPORT / EXPORT DATA
# =========================================================

data = pd.read_excel(
    r"simple_model\PR2\voltageSensitivity\BK28_pole_wise_load_solar_real_before_reconstructed_final.xlsx", sheet_name="Pole_Data_With_Total_Data"
)

# If Timestamp is first column
timestamps = data.iloc[:, 0]

# Column AE and AF
# AE = column 30 (0-based index 30)
# AF = column 31 (0-based index 31)
# Adjust if needed

P_import = data.iloc[:, 30].values   # Total Import (kW)
P_export = data.iloc[:, 31].values   # Total Export (kW)

T = len(P_import)

# =========================================================
# 2. NATIONAL DEMAND DATA (MW → kW)
# =========================================================

nat_data = {
    "Net_Load_MW": [
1580.4,1585.2,1565,1550.3,1525.6,1494.1,1491.5,1483.4,
1464,1458.8,1440.9,1449.9,1436.7,1428.7,1426.3,1444,
1449,1500.6,1579.1,1639.9,1721.7,1849.5,1992.4,2058.1,
2067.8,2119.2,2100.4,2026.6,1891.2,1788.1,1702.5,1642.9,
1574.3,1520.5,1449.5,1411,1321.9,1246.5,1202,1180.1,
1124.5,1103.4,1098.1,1090.6,1076.7,1096.3,1108,1120.3,
1105.6,1033.4,1003.2,992.4,962.6,929.2,888.7,904.3,
937.7,1017.8,1088.4,1199.1,1274.4,1353.2,1501.2,1604,
1734,1854.3,1962,2054,2145.2,2215.1,2267.7,2330.3,
2358.8,2456.8,2588.5,2689.9,2702.3,2676,2640.5,2606,
2551.2,2501.1,2445,2393.4,2337.2,2269.3,2205.8,2142.4,
2073.1,2001.3,1942.7,1904.5,1856.4,1807.8,1764.4,1737.3
] * 2
}

P_nat_kW = np.array(nat_data["Net_Load_MW"]) * 1000

# Match lengths
min_len = min(T, len(P_nat_kW))
P_import = P_import[:min_len]
P_export = P_export[:min_len]
P_nat_kW = P_nat_kW[:min_len]
timestamps = timestamps[:min_len]

# =========================================================
# 3. RECONSTRUCTION
# =========================================================

net = P_import - P_export

# Night indices (no export)
night_idx = np.where(P_export == 0)[0]

# Scaling factor alpha
alpha = np.mean(P_import[night_idx] / P_nat_kW[night_idx])

# Initialize arrays
P_load = np.zeros(min_len)
P_solar = np.zeros(min_len)

for t in range(min_len):

    if P_export[t] == 0:
        # At night → measured import = true load
        P_load[t] = P_import[t]
    else:
        # Daytime → scale from national demand
        P_load[t] = alpha * P_nat_kW[t]

    # Solar extraction
    P_solar[t] = P_load[t] - net[t]

    # Physical constraint
    if P_solar[t] < 0:
        P_solar[t] = 0

# =========================================================
# 4. SAVE RESULTS
# =========================================================

output = pd.DataFrame({
    "Timestamp": timestamps,
    "Total_Load_kW": P_load,
    "Total_Solar_kW": P_solar
})

output.to_excel("Reconstructed_Total_Load_Solar.xlsx", index=False)

alpha_df = pd.DataFrame({
    "Alpha": [alpha]
})

alpha_df.to_excel("Scaling_Factor_Alpha_Total.xlsx", index=False)

print("Reconstruction completed.")
print("Files saved:")
print(" - Reconstructed_Total_Load_Solar.xlsx")
print(" - Scaling_Factor_Alpha_Total.xlsx")




# =========================================================
# PLOT RESULTS
# =========================================================

time = range(min_len)   # or use timestamps if they are datetime

plt.figure(figsize=(14,6))

plt.plot(time, P_import, label="Import (given)")
plt.plot(time, P_export, label="Export (given)")
plt.plot(time, P_load, label="Reconstructed Load")
plt.plot(time, P_solar, label="Reconstructed Solar")

plt.xlabel("Time Step")
plt.ylabel("Power (kW)")
plt.title("Total Import / Export vs Reconstructed Load and Solar")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()