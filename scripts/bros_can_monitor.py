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


FPS = 15

MSG_FROM_BROS = {
    "XVR_Status": MissionNodeMsgPorts.KSICHT_STATUS,
    "XVR_SetpointsMotor_A": MissionNodeMsgPorts.WHEEL_SPEED_CMD,
    "XVR_Control": MissionNodeMsgPorts.STEERING_ANGLE_CMD
}

MSG_TO_BROS_CAN1 = {
    "MCR_ActualValues_A": CAN1NodeMsgPorts.WHEEL_SPEED,
    "SA_SteeringAngle": CAN1NodeMsgPorts.STEERING_ANGLE,
    "DSH_Status-MISSION": CAN1NodeMsgPorts.MISSION,
    "DSH_Status-START_BUTTON": CAN1NodeMsgPorts.START_BUTTON
}

MSG_TO_BROS_CAN2 = {
    "RES_Status": CAN2NodeMsgPorts.GO_SIGNAL,
    "INS_D_EKF_EULER": CAN2NodeMsgPorts.POSITION,
    "INS_D_EKF_POS": CAN2NodeMsgPorts.EULER,
    "INS_D_EKF_VEL_BODY": CAN2NodeMsgPorts.ACCELERATION
}


def create_bros_sender_sockets():
    sender_sockets = {}
    for key, value in MSG_FROM_BROS.items():
        sender_sockets[key] = create_subscriber_socket(value)
    return sender_sockets


def create_bros_receive_sockets_can1():
    recv_sockets_can1 = {}
    for key, value in MSG_TO_BROS_CAN1.items():
        recv_sockets_can1[key] = create_subscriber_socket(value)
    return recv_sockets_can1


def create_bros_receive_sockets_can2():
    recv_sockets_can2 = {}
    for key, value in MSG_TO_BROS_CAN2.items():
        recv_sockets_can2[key] = create_subscriber_socket(value)
    return recv_sockets_can2


def loop(window: curses.window):
    curses.initscr()
    curses.start_color()
    sender_sockets = create_bros_sender_sockets()
    recv_sockets_can1 = create_bros_receive_sockets_can1()
    recv_sockets_can2 = create_bros_receive_sockets_can2()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)
    while True:
        start = time.perf_counter()
        window.addstr(0, 0, "BROS CAN COMMUNICATION MONITOR\n", curses.color_pair(1))
        window.addstr(2, 5, "Sending on CAN1:\n", curses.color_pair(3))

        sender_data = ""
        for data_sent in MSG_FROM_BROS:
            sender_data += f"\t{data_sent}: "
            sender_data += str(get_last_subscription_data(sender_sockets[data_sent])) + "\n"
        window.addstr(3, 5, sender_data)

        window.addstr(8, 5, "Receiving on CAN1:\n", curses.color_pair(2))

        recv_data_can1 = ""
        for data_recvd in MSG_TO_BROS_CAN1:
            recv_data_can1 += f"\t{data_recvd}: "
            recv_data_can1 += str(get_last_subscription_data(recv_sockets_can1[data_recvd])) + "\n"
        window.addstr(9, 5, recv_data_can1)

        window.addstr(14, 5, "Receiving on CAN2:\n", curses.color_pair(2))

        recv_data_can2 = ""
        for data_recvd in MSG_TO_BROS_CAN2:
            recv_data_can2 += f"\t{data_recvd}: "
            recv_data_can2 += str(get_last_subscription_data(recv_sockets_can2[data_recvd])) + "\n"
        window.addstr(15, 5, recv_data_can2)

        delta = time.perf_counter() - start
        window.refresh()
        time_to_sleep = (1. / FPS) - delta
        if time_to_sleep > 0.:
            curses.delay_output(int(time_to_sleep * 1000))


curses.wrapper(loop)
