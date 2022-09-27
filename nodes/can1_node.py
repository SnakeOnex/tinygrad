import multiprocessing as mp
import multiprocessing.connection as connection
from multiprocessing import shared_memory
from enum import Enum

import time
import math

from pycandb.can_interface import CanInterface

class Can1RecvItems(Enum):
    wheel_speed = 0
    steering_actual = 1

class Can1SendItems(Enum):
    steering = 0
    engine_force = 1

class Can1Node(mp.Process):
    def __init__(self, mode, recv_name):
        mp.Process.__init__(self)
        self.mode = mode # "SIMULATION", "CAN"
        self.bus_name = "can1"
        self.report_rate = 100 # hz

        self.recv_name = recv_name

    def initialize(self):
        print("CAN1 INIT")
        self.can1_recv_state = shared_memory.ShareableList(name=self.recv_name)
        self.CAN1 = CanInterface("data/D1.json", 0, False)


        self.message_callbacks = {
            self.CAN1.name2id["MCR_ActualValues_A"]: self.receive_MCR_ActualValues_A,
        }

    def run(self):
        self.initialize()

        while True:
            msg = self.CAN1.recv_can_msg()
            # print("msg: ", msg)

            if msg.arbitration_id in self.message_callbacks:
                values = self.CAN1.read_can_msg(msg)

                self.message_callbacks[msg.arbitration_id](values)

    ## CAN MESSAGE RECEIVE CALLBACK FUNCTIONS
    def receive_MCR_ActualValues_A(self, values):
        WHEEL_SPEED_TO_MS = 1 / (60 * 6.7) * 2 * 0.2 * math.pi
        
        wheel_speed = float(values[3]) * WHEEL_SPEED_TO_MS
        self.can1_recv_state[Can1RecvItems.wheel_speed.value] = wheel_speed