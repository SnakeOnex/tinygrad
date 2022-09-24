from enum import IntEnum

# Created at : 06/13/22 10:51:56
CREATE_DATE = "1655110316.919314"


class CanIds(IntEnum):
    # Send messages
    MCF_MasterControlStatus = 0x18ff6080
    MCF_SetpointsMotor_A = 0x18ff6180
    MCF_SetpointsMotor_B = 0x18ff6280
    MCR_MasterControlStatus = 0x18ff1080
    MCR_SetpointsMotor_A = 0x18ff1180
    MCR_SetpointsMotor_B = 0x18ff1280
    SA_Control = 0x150
    XVR_Status = 0x21
    # XVR_Time_Sync_CAN1 = 0x668
    DTL_DV_driving_dynamics_1 = 0x500
    DTL_DV_driving_dynamics_2 = 0x501
    DTL_DV_system_status = 0x502
    RES_NMT_Mode_Control = 0x0
    XVR_CPU_STATUS = 0x666
    XVR_EnableLidar = 0x22
    XVR_RAM_GPU_STATUS = 0x667
    XVR_Time_Sync_CAN2 = 0x669
    # Received messages
    AMS_OperatingLimits = 0x192
    AMS_Status = 0x90
    DSH_Status = 0x3c0
    EBSS_AirPressure = 0xc1
    EBSS_Status = 0xa0
    MCF_ActualValues_A = 0x18ff01ef
    MCF_ActualValues_B = 0x18ff02ef
    MCF_DeviceStatus = 0x18ff00ef
    MCR_ActualValues_A = 0x18ff01ea
    MCR_ActualValues_B = 0x18ff02ea
    MCR_DeviceStatus = 0x18ff00ea
    PDL_Accelerator = 0x140
    PDL_Brakes = 0x142
    SA_SteeringAngle = 0x151
    XVR_Time_Sync_CAN1 = 0x668
    AMS_ACPMeasurements = 0x390
    INS_D_AIR_DATA_ALTITUDE = 0x163
    INS_D_AUTO_TRACK_SLIP_CURV = 0x220
    INS_D_EKF_ALTITUDE = 0x135
    INS_D_EKF_EULER = 0x132
    INS_D_EKF_INFO = 0x130
    INS_D_EKF_ORIENT_ACC = 0x133
    INS_D_EKF_POS = 0x134
    INS_D_EKF_POS_ACC = 0x136
    INS_D_EKF_QUAT = 0x131
    INS_D_EKF_VEL_ACC = 0x138
    INS_D_EKF_VEL_BODY = 0x139
    INS_D_EKF_VEL_NED = 0x137
    INS_D_GPS1_COURSE = 0x173
    INS_D_GPS1_HDT = 0x179
    INS_D_GPS1_HDT_INFO = 0x178
    INS_D_GPS1_POS = 0x175
    INS_D_GPS1_POS_ACC = 0x177
    INS_D_GPS1_POS_ALT = 0x176
    INS_D_GPS1_POS_INFO = 0x174
    INS_D_GPS1_VEL = 0x171
    INS_D_GPS1_VEL_ACC = 0x172
    INS_D_GPS1_VEL_INFO = 0x170
    INS_D_GPS2_COURSE = 0x183
    INS_D_GPS2_HDT = 0x189
    INS_D_GPS2_HDT_INFO = 0x188
    INS_D_GPS2_POS = 0x185
    INS_D_GPS2_POS_ACC = 0x187
    INS_D_GPS2_POS_ALT = 0x186
    INS_D_GPS2_POS_INFO = 0x184
    INS_D_GPS2_VEL = 0x181
    INS_D_GPS2_VEL_ACC = 0x182
    INS_D_GPS2_VEL_INFO = 0x180
    INS_D_IMU_ACCEL = 0x121
    INS_D_IMU_GYRO = 0x122
    INS_D_IMU_INFO = 0x120
    INS_D_ODO_INFO = 0x160
    INS_D_ODO_VEL = 0x161
    INS_D_PRESSURE_ALTITUDE = 0x16f
    INS_D_PRESSURE_INFO = 0x16e
    INS_D_STATUS_01 = 0x99
    INS_D_STATUS_02 = 0x101
    INS_D_STATUS_03 = 0x103
    INS_D_UTC_0 = 0x110
    INS_D_UTC_1 = 0x111
    RES_Emergency = 0x91
    RES_Status = 0x191
    #XVR_Time_Sync_CAN2 = 0x669


CAN_0_FILTER = 0x7ff
CAN_1_FILTER = 0x18ff07ff
