import pandas as pd
import numpy as np

# =========================================================
# 1. READ YOUR 14-POLE DATA
# =========================================================

data = pd.read_excel(
    r"simple_model\PR2\voltageSensitivity\BK28_pole_wise_load_solar_real_before_reconstructed_final_m.xlsx",sheet_name="Pole_Data_With_Total_Data"
)

timestamps = data.iloc[:,0]

P_import = data.iloc[:,1:15].values   # shape (T,14) in kW
P_export = data.iloc[:,15:29].values # shape (T,14) in kW

T, N = P_import.shape  # T timesteps, N=14 poles

# =========================================================
# 2. NATIONAL DEMAND DATA (MW → convert to kW)
# =========================================================

nat_data = {
"Timestamp": [
# ---- paste timestamps if needed ----
],
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
]*2   # repeated for 2 days
}

P_nat_MW = np.array(nat_data["Net_Load_MW"])
P_nat_kW = P_nat_MW * 1000  # convert to kW

# Ensure same length
min_len = min(T, len(P_nat_kW))
P_nat_kW = P_nat_kW[:min_len]
P_import = P_import[:min_len, :]
P_export = P_export[:min_len, :]
timestamps = timestamps[:min_len]

# =========================================================
# 3. INITIALIZE OUTPUT MATRICES
# =========================================================

P_load = np.zeros_like(P_import)
P_solar = np.zeros_like(P_import)
alpha_values = np.zeros(N)

# =========================================================
# 4. LOOP THROUGH EACH POLE
# =========================================================

for i in range(N):

    imp = P_import[:, i]
    exp = P_export[:, i]
    net = imp - exp

    # Night indices (export == 0)
    night_idx = np.where(exp == 0)[0]

    # Compute alpha for this pole
    alpha = np.mean(imp[night_idx] / P_nat_kW[night_idx])
    alpha_values[i] = alpha

    # Reconstruct load
    for t in range(min_len):
        if exp[t] == 0:
            P_load[t, i] = imp[t]          # measured load at night
        else:
            P_load[t, i] = alpha * P_nat_kW[t]

    # Extract solar
    P_solar[:, i] = P_load[:, i] - net
    P_solar[P_solar[:, i] < 0, i] = 0   # enforce physical constraint

# =========================================================
# 5. SAVE TO EXCEL
# =========================================================

output = pd.DataFrame()
output["Timestamp"] = timestamps

# Add load columns
for i in range(N):
    output[f"Pole_{i+1}_Load_kW"] = P_load[:, i]

# Add solar columns
for i in range(N):
    output[f"Pole_{i+1}_Solar_kW"] = P_solar[:, i]

# Save
output.to_excel("Reconstructed_14Pole_Load_Solar.xlsx", index=False)

# Save alpha separately
alpha_df = pd.DataFrame({
    "Pole": [f"Pole_{i+1}" for i in range(N)],
    "Alpha": alpha_values
})
alpha_df.to_excel("Scaling_Factors_Alpha.xlsx", index=False)

print("Reconstruction completed.")
print("Files saved:")
print(" - Reconstructed_14Pole_Load_Solar.xlsx")
print(" - Scaling_Factors_Alpha.xlsx")