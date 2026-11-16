# PX4 parameters from the two logged Offboard flights (log_70, log_71)

**Read this first (added 2026-11-16).** The vehicles fly `EKF2_EV_CTRL 11` and that stays. The height reference is the downward rangefinder, not vision and not the barometer: `EKF2_HGT_REF 2`, `EKF2_RNG_CTRL 2`, `EKF2_BARO_CTRL 0`, `MPC_ALT_MODE 2`, the same on all four vehicles (read from each vehicle's latest log on 2026-11-16). PX4 therefore holds the height above the ground: `dist_bottom` is 0.10 m standing and 1.00 m (0.87 to 1.11) in flight, valid for the whole of both Nov 13 flights. Vision height is a secondary source. A simulation that takes its height from vision or the barometer will not follow the ground the way the vehicles do.

PX4 v1.15.4, Cube Orange+. Values at log start; no parameter changed during either log. Full dumps: `log_70_params.txt`, `log_71_params.txt`.

| Parameter | log_70 | log_71 |
|---|---|---|
| EKF2_EV_CTRL | 11 | 11 |
| EKF2_EV_DELAY | 50 | 50 |
| EKF2_EV_QMIN | 0 | 0 |
| EKF2_EV_NOISE_MD | 0 | 0 |
| EKF2_EVP_NOISE | 0.1 | 0.1 |
| EKF2_EVV_NOISE | 0.1 | 0.1 |
| EKF2_EVA_NOISE | 0.0500909 | 0.0500909 |
| EKF2_EVP_GATE | 5 | 5 |
| EKF2_EVV_GATE | 3 | 3 |
| EKF2_EV_POS_X | 0.15 | 0.15 |
| EKF2_EV_POS_Y | 0 | 0 |
| EKF2_EV_POS_Z | 0 | 0 |
| EKF2_HGT_REF | 2 | 2 |
| EKF2_RNG_CTRL | 2 | 2 |
| EKF2_RNG_NOISE | 0.1 | 0.1 |
| EKF2_RNG_SFE | 0.05 | 0.05 |
| EKF2_RNG_GATE | 5 | 5 |
| EKF2_RNG_POS_X | 0 | 0 |
| EKF2_RNG_POS_Y | 0 | 0 |
| EKF2_RNG_POS_Z | 0 | 0 |
| EKF2_RNG_A_HMAX | 7 | 7 |
| EKF2_RNG_A_VMAX | 1 | 1 |
| EKF2_RNG_DELAY | 5 | 5 |
| EKF2_MIN_RNG | 0.1 | 0.1 |
| EKF2_BARO_CTRL | 0 | 0 |
| EKF2_MAG_TYPE | 5 | 5 |
| EKF2_MAG_DECL | -5.42656 | -5.42656 |
| EKF2_DECL_TYPE | 3 | 3 |
| EKF2_GPS_CTRL | 15 | 15 |
| EKF2_GPS_DELAY | 110 | 110 |
| EKF2_REQ_GPS_H | 10 | 10 |
| EKF2_REQ_EPH | 3 | 3 |
| EKF2_REQ_NSATS | 6 | 6 |
| MPC_XY_VEL_MAX | 0.5 | 0.5 |
| MPC_XY_CRUISE | 0.5 | 0.5 |
| MPC_ACC_HOR | 2 | 2 |
| MPC_ACC_HOR_MAX | 2 | 2 |
| MPC_JERK_AUTO | 2 | 2 |
| MPC_JERK_MAX | 8 | 8 |
| MPC_XY_P | 2 | 2 |
| MPC_Z_P | 1.45 | 1.45 |
| MPC_XY_ERR_MAX | 1.5 | 1.5 |
| MPC_YAWRAUTO_MAX | 20 | 20 |
| MPC_YAWRAUTO_ACC | 10 | 10 |
| MPC_Z_VEL_MAX_UP | 1.5 | 1.5 |
| MPC_Z_VEL_MAX_DN | 1.5 | 1.5 |
| MPC_Z_V_AUTO_UP | 1.5 | 1.5 |
| MPC_Z_V_AUTO_DN | 1.5 | 1.5 |
| MPC_TKO_SPEED | 1 | 1 |
| MPC_LAND_SPEED | 0.6 | 0.6 |
| MPC_LAND_ALT1 | 1 | 1 |
| MPC_LAND_ALT2 | 1 | 1 |
| MPC_LAND_ALT3 | 1 | 1 |
| MPC_LAND_CRWL | 0.3 | 0.3 |
| MPC_THR_HOVER | 0.5 | 0.5 |
| MPC_USE_HTE | 1 | 1 |
| COM_ARM_EKF_VEL | 0.5 | 0.5 |
| COM_ARM_EKF_POS | 0.5 | 0.5 |
| COM_ARM_EKF_HGT | 1 | 1 |
| COM_ARM_EKF_YAW | 0.5 | 0.5 |
| COM_OF_LOSS_T | 1 | 1 |
| COM_OBL_RC_ACT | 0 | 0 |
| COM_RC_LOSS_T | 2 | 2 |
| COM_RCL_EXCEPT | 6 | 6 |
| COM_POS_FS_EPH | 5 | 5 |
| COM_VEL_FS_EVH | 1 | 1 |
| COM_POS_LOW_EPH | -1 | -1 |
| COM_FAIL_ACT_T | 5 | 5 |
| COM_DL_LOSS_T | 10 | 10 |
| NAV_RCL_ACT | 1 | 1 |
| NAV_DLL_ACT | 0 | 0 |
| COM_LOW_BAT_ACT | 0 | 0 |
| SENS_BOARD_X_OFF | -1.79536 | -1.79536 |
| SENS_BOARD_Y_OFF | -0.18263 | -0.18263 |
| SENS_BOARD_Z_OFF | 0 | 0 |
| SENS_BOARD_ROT | 0 | 0 |
| UXRCE_DDS_DOM_ID | 0 | 0 |
| SYS_HAS_MAG | 1 | 1 |
| SYS_HAS_GPS | 1 | 1 |

Not present in this firmware build: EKF2_AGP_CTRL, EKF2_AGP_DELAY, EKF2_AGP_NOISE, EKF2_AGP_GATE
