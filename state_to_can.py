import math

## CAN1 SEND
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

    values = [act_InverterStatus, act_InverterReady, act_InverterErrorStatus, act_Speed, act_Torque, act_Power, act_MotorTemperature]
    return values

can1_send_callbacks = {
    "MCR_ActualValues_A" : state_to_MCR_ActualValues_A
}

## CAN1 RECV

def receive_XVR_Control(state, values):
    state.steering_angle = values[0]

can1_recv_callbacks = {
    "XVR_Control" : receive_XVR_Control
}