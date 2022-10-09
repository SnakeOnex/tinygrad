import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import sys
import math
import time

from nodes.can1_node import Can1RecvItems, Can1SendItems
from nodes.can2_node import Can2RecvItems, Can2SendItems

from missions.trackdrive import Trackdrive
from nodes.asm import ASM, AS

from pycandb.can_interface import CanInterface

class MissionNode(mp.Process):
    def __init__(self, perception_out, can1_recv_name, can2_recv_name):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_name = can1_recv_name
        self.can2_recv_name = can2_recv_name

        self.frequency = 10 # Hz

        # Autonomous State Machine
        self.ASM = ASM()

    def initialize(self):
        self.can1_recv_state = shared_memory.ShareableList(name=self.can1_recv_name)
        self.can2_recv_state = shared_memory.ShareableList(name=self.can2_recv_name)
        self.mission = Trackdrive(self.perception_out, self.can1_recv_state)

        self.CAN1 = CanInterface("data/D1.json", 0, False)

    def run(self):
        self.initialize()

        while True:
            start_time = time.perf_counter()

            # 1. update AS State
            self.ASM.update(go_signal=self.can2_recv_state[Can2RecvItems.go_signal.value])

            print("AS: ", self.ASM.AS)

            if self.ASM.AS == AS.DRIVING:
                steering_angle, speed = self.mission.loop()

                self.CAN1.send_can_msg([steering_angle], self.CAN1.name2id["XVR_Control"])
                self.CAN1.send_can_msg([0, 0, 0, 0, speed, 0], self.CAN1.name2id["XVR_SetpointsMotor_A"])

            end_time = time.perf_counter()
            # print("loop_delta: ", end_time - start_time)
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)

            if time_to_sleep > 0.:
                time.sleep(time_to_sleep)