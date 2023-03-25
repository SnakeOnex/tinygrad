import time
import os
import numpy as np
import curses

import sys
sys.path.append("..")

from config import MissionNodeMsgPorts
from config import CAN1NodeMsgPorts
from config import CAN2NodeMsgPorts
from internode_communication import create_subscriber_socket, get_last_subscription_data


FPS = 30

MSG_FROM_BROS = {
    "XVR_Status": MissionNodeMsgPorts.KSICHT_STATUS,
    "XVR_SetpointsMotor_A": MissionNodeMsgPorts.WHEEL_SPEED_CMD,
    "XVR_Control": MissionNodeMsgPorts.STEERING_ANGLE_CMD
}

MSG_TO_BROS_CAN1 = {
    "MCR_ActualValues_A": CAN1NodeMsgPorts.WHEEL_SPEED,
    "SA_SteeringAngle": CAN1NodeMsgPorts.STEERING_ANGLE,
    "DSH_Status - MISSION": CAN1NodeMsgPorts.MISSION,
    "DSH_Status - START_BUTTON": CAN1NodeMsgPorts.START_BUTTON
}

MSG_TO_BROS_CAN2 = {
    "RES_Status": CAN2NodeMsgPorts.GO_SIGNAL,
    "INS_D_EKF_EULER": CAN2NodeMsgPorts.POSITION,
    "INS_D_EKF_POS": CAN2NodeMsgPorts.EULER,
    "INS_D_EKF_VEL_BODY": CAN2NodeMsgPorts.ACCELERATION
}


def create_sockets_from_dict(socket_dict):
    sockets = {}
    for key, value in socket_dict.items():
        sockets[key] = create_subscriber_socket(value)
    return sockets


def update_data_dict(dest_dict, socket_dict):
    for data_field, socket in socket_dict.items():
        updated_data = get_last_subscription_data(socket)
        if updated_data != None:
            dest_dict[data_field] = np.array2string(np.array(updated_data), precision=3, suppress_small=False, floatmode="fixed", sign=" ")


def data_dict_to_str(data_dict):
    data_str = ""
    for data_field, data_val in data_dict.items():
        data_str += f"\n\t{data_field}: {data_val}\n"
    return data_str


def loop(window: curses.window):
    curses.initscr()
    curses.start_color()

    sender_sockets = create_sockets_from_dict(MSG_FROM_BROS)
    recv_sockets_can1 = create_sockets_from_dict(MSG_TO_BROS_CAN1)
    recv_sockets_can2 = create_sockets_from_dict(MSG_TO_BROS_CAN2)

    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)

    data_sent = dict.fromkeys(MSG_FROM_BROS.keys(), None)
    data_recvd_can1 = dict.fromkeys(MSG_TO_BROS_CAN1.keys(), None)
    data_recvd_can2 = dict.fromkeys(MSG_TO_BROS_CAN2.keys(), None)

    while True:
        start = time.perf_counter()

        update_data_dict(data_sent, sender_sockets)
        update_data_dict(data_recvd_can1, recv_sockets_can1)
        update_data_dict(data_recvd_can2, recv_sockets_can2)

        try:
            window.addstr(0, 0, "BROS CAN MONITOR\n", curses.color_pair(1))
            window.addstr(2, 5, "Sending on CAN1:\n", curses.color_pair(3))
            window.addstr(3, 5, data_dict_to_str(data_sent))
            window.addstr(11, 5, "Receiving on CAN1:\n", curses.color_pair(2))
            window.addstr(12, 5, data_dict_to_str(data_recvd_can1))
            window.addstr(21, 5, "Receiving on CAN2:\n", curses.color_pair(2))
            window.addstr(22, 5, data_dict_to_str(data_recvd_can2))

        except Exception as e:

            if e == curses.ERR:
                print("Terminal too small!")
                sys.exit(1)

        delta = time.perf_counter() - start
        window.refresh()

        time_to_sleep = (1. / FPS) - delta
        if time_to_sleep > 0.:
            curses.delay_output(int(time_to_sleep * 1000))


curses.wrapper(loop)
