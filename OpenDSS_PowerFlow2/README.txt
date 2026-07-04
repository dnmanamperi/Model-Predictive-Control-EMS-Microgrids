This package is the corrected transformer + other-feeders + feeder-2-head-matching model
using the REAL battery parameters provided by the user:

- Pch,max = Pdis,max = 50 kW
- ηch = ηdis = 95%
- SOCmin = 20%
- SOCmax = 80%
- initial SOC = 50%

Important note:
This model is for PHYSICAL FEASIBILITY. It will generally NOT match AD/AE exactly
if the EMS schedule assumed an ideal battery without these limits.
If exact AD/AE replay is needed, use the load/gen split battery model instead.

Run:
1. Put MPC_Results_Fixed_Fixed_Fixed.xlsx in this folder.
2. Compile Master_XFMR_OtherFeeders_HeadMatch_RealBattery.dss once in OpenDSS.
3. In MATLAB run:
   replay_xfmr_otherfeeders_headmatch_storage_realbattery_FIXED
