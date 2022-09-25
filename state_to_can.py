from can.can_interface import CanInterface

## CAN1
def state_to_MCR_ActualValues_A(state, message):
    act_InverterStatus = 0
    act_InverterReady = 0
    act_InverterErrorStatus = 0.
    act_Speed = state.speed
    act_Torque = 0
    act_Power = 0
    act_MotorTemperature = 0

    values = [act_InverterStatus, act_InverterReady, act_InverterErrorStatus, act_Speed, act_Torque, act_Power, act_MotorTemperature]
