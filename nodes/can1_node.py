import multiprocessing as mp
import multiprocessing.connection as connection
from enum import Enum

import time
import math
import pickle

from pycandb.can_interface import CanInterface

from internode_communication import create_publisher_socket, publish_data
from config import CAN1NodeMsgPorts


class Can1Node(mp.Process):
    def __init__(self, mode):
        mp.Process.__init__(self)
        self.mode = mode  # "SIM", "CAN"
        self.bus_name = "can1"
        self.report_rate = 100  # hz

    def initialize(self):
        self.CAN1 = CanInterface("data/D1.json", 0, False)

        self.wheel_speed_socket = create_publisher_socket(CAN1NodeMsgPorts.WHEEL_SPEED)
        self.mission_socket = create_publisher_socket(CAN1NodeMsgPorts.MISSION)
        self.start_button_socket = create_publisher_socket(CAN1NodeMsgPorts.START_BUTTON)

        self.message_callbacks = {
            self.CAN1.name2id["MCR_ActualValues_A"]: self.receive_MCR_ActualValues_A,
            self.CAN1.name2id["DSH_Status"]: self.receive_DSH_Status,
        }

    def run(self):
        self.initialize()

        while True:
            msg = self.CAN1.recv_can_msg()

            if msg.arbitration_id in self.message_callbacks:
                values = self.CAN1.read_can_msg(msg)

                self.message_callbacks[msg.arbitration_id](values)

    # CAN MESSAGE RECEIVE CALLBACK FUNCTIONS
    def receive_MCR_ActualValues_A(self, values):
        WHEEL_SPEED_TO_MS = 1 / (60 * 6.7) * 2 * 0.2 * math.pi

        wheel_speed = float(values[3]) * WHEEL_SPEED_TO_MS
        publish_data(self.wheel_speed_socket, wheel_speed)

    def receive_DSH_Status(self, values):
        mission = values[3]
        start_button = values[5]
        publish_data(self.mission_socket, mission)
        publish_data(self.start_button_socket, start_button)
