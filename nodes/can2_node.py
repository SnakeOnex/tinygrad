import multiprocessing as mp
import multiprocessing.connection as connection
from multiprocessing import shared_memory
from enum import Enum

import time
import math
import pickle

from pycandb.can_interface import CanInterface

from internode_communication import create_publisher_socket, publish_data
from config import CAN2NodeMsgPorts

class Can2RecvItems(Enum):
    go_signal = 0
    steering_actual = 1

class Can2SendItems(Enum):
    steering = 0
    engine_force = 1

class Can2Node(mp.Process):
    def __init__(self, mode):
        mp.Process.__init__(self)
        self.mode = mode # "SIMULATION", "CAN"
        self.bus_name = "can2"

    def initialize(self):
        self.CAN2 = CanInterface("data/D1.json", 1, False)

        self.go_signal_socket = create_publisher_socket(CAN2NodeMsgPorts.GO_SIGNAL)

        self.message_callbacks = {
            self.CAN2.name2id["RES_Status"]: self.receive_RES_Status,
        }

    def run(self):
        self.initialize()

        while True:
            msg = self.CAN2.recv_can_msg()

            if msg.arbitration_id in self.message_callbacks:
                values = self.CAN2.read_can_msg(msg)
                # print(f"received msg: {self.CAN2.id2name[msg.arbitration_id]} values: {values}")
                self.message_callbacks[msg.arbitration_id](values)

    ## CAN MESSAGE RECEIVE CALLBACK FUNCTIONS
    def receive_RES_Status(self, values):
        # print(f"received RES status, values: ", values)
        go_signal = values[2]
        # print("go_signal: ", go_signal)
        publish_data(self.go_signal_socket, go_signal)