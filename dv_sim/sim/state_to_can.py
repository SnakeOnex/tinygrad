import math

from .state import AS

# CAN1 SEND

def state_to_MCR_ActualValues_A(state):
    act_InverterStatus = 0
    act_InverterReady = 0
    act_InverterErrorStatus = 0.

    # (1 / (60 * GEAR_RATIO)) * (2 * PI * 0.2)
    WHEEL_SPEED_TO_MS = 1 / (60 * 6.7) * 2 * 0.2 * math.pi
    act_Speed = int(state.speed / WHEEL_SPEED_TO_MS)

    act_Torque = 0
    act_Power = 0
    act_MotorTemperature = 0

    values = [act_InverterStatus, act_InverterReady, act_InverterErrorStatus,
              act_Speed, act_Torque, act_Power, act_MotorTemperature]
    return values

def state_to_DSH_Status(state):
    Voltage = 0
    Current = 0
    Brightness = 0
    Missions_sel = state.mission  # TODO: change to state.mission
    TSON_INT = 0
    START_INT = 1
    SW1 = 0
    SW2 = 0
    SW3 = 0
    SDC_in = 0
    MCU_temperature = 0

    values = [Voltage, Current, Brightness, Missions_sel, TSON_INT, START_INT, SW1, SW2, SW3, SDC_in, MCU_temperature]

    return values

def state_to_SA_SteeringAngle(state):
    Angle_FT = 0
    AngularSpeed_FT = 0
    Angle = state.steering_angle*4.0
    AngularSpeed = 0
    SEQ = 0

    values = [Angle_FT, AngularSpeed_FT, Angle, AngularSpeed, SEQ]

    return values

def state_to_EBSS_Status(state):
    if state.emergency_signal:
        CarState = 0
    else:
        CarState = 5

    values = [0 for _ in range(34)]
    values[0] = CarState
    values[3] = 1 # tson button
    values[9] = 1 # asms out

    return values

# CAN2 SEND
    
def state_to_RES_Status(state):
    E_stop = 0
    Switch = 0

    Go_Signal = state.go_signal

    Object_2000h = 0
    Object_2001h = 0
    Object_2002h = 0
    Object_2003h = 0
    E_Stop_2 = 0
    Object_2004h = 0
    Object_2005h = 0
    Radio_Quality = 0
    TX_to_RX = 0
    RX_to_TX = 0
    Battery = 0
    Command = 0
    PreAlarmInterrupt = 0

    values = [E_stop, Switch, Go_Signal, Object_2000h, Object_2001h, Object_2002h, Object_2003h, E_Stop_2, Object_2004h,
              Object_2005h, Radio_Quality, TX_to_RX, RX_to_TX, Battery, Command, PreAlarmInterrupt]

    return values


def state_to_INS_D_EKF_POS(state):
    values = [state.car_pos[0], state.car_pos[1]]
    # print(values)
    return values


def state_to_INS_D_EKF_EULER(state):
    # subtract 180 to change range to < -180, 180 > to mimick INS
    values = [0., 0., state.heading-180]
    return values

def state_to_INS_D_EKF_VEL_BODY(state):
    vel_x, vel_y = state.velocity
    values = [vel_x, vel_y, 0.]
    return values


can1_send_callbacks = {
    "MCR_ActualValues_A": state_to_MCR_ActualValues_A,
    "DSH_Status": state_to_DSH_Status,
    "SA_SteeringAngle": state_to_SA_SteeringAngle,
    "EBSS_Status": state_to_EBSS_Status
}

can2_send_callbacks = {
    "RES_Status": state_to_RES_Status,
    "INS_D_EKF_POS": state_to_INS_D_EKF_POS,
    "INS_D_EKF_EULER": state_to_INS_D_EKF_EULER,
    "INS_D_EKF_VEL_BODY": state_to_INS_D_EKF_VEL_BODY
}

# CAN1 RECV


def receive_XVR_Control(state, values):
    # state.steering_angle = values[0]
    state.steering_angle_set_point = values[0]/4.0


def receive_XVR_SetpointsMotor_A(state, values):
    state.speed_set_point = values[4]
    # print("received set point: ", state.speed_set_point)


def receive_XVR_Status(state, values):
    state.AS = AS(values[0])


can1_recv_callbacks = {
    "XVR_Status": receive_XVR_Status,
    "XVR_Control": receive_XVR_Control,
    "XVR_SetpointsMotor_A": receive_XVR_SetpointsMotor_A
}
