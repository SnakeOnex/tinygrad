import multiprocessing as mp
import multiprocessing.connection as connection
from multiprocessing import shared_memory
from enum import Enum

import time
import math

from pycandb.can_interface import CanInterface

class Can2RecvItems(Enum):
    go_signal = 0
    steering_actual = 1

class Can2SendItems(Enum):
    steering = 0
    engine_force = 1

class Can2Node(mp.Process):
    def __init__(self, mode, recv_name):
        mp.Process.__init__(self)
        self.mode = mode # "SIMULATION", "CAN"
        self.bus_name = "can2"
        self.recv_name = recv_name

    def initialize(self):
        self.can2_recv_state = shared_memory.ShareableList(name=self.recv_name)
        self.CAN2 = CanInterface("data/D1.json", 1, False)

        self.message_callbacks = {
            self.CAN2.name2id["RES_Status"]: self.receive_RES_Status,
        }

    def run(self):
        self.initialize()

        while True:
            msg = self.CAN2.recv_can_msg()

            # print("received msg.id: ", msg.arbitration_id)
            # print("CAN_msg: ", msg)

            if msg.arbitration_id in self.message_callbacks:
                values = self.CAN2.read_can_msg(msg)
                # print(f"received msg: {self.CAN2.id2name[msg.arbitration_id]} values: {values}")
                self.message_callbacks[msg.arbitration_id](values)

    ## CAN MESSAGE RECEIVE CALLBACK FUNCTIONS
    def receive_RES_Status(self, values):
        # print(f"received RES status, values: ", values)
        go_signal = values[2]
        # print("go_signal: ", go_signal)
        self.can2_recv_state[Can2RecvItems.go_signal.value] = go_signal;