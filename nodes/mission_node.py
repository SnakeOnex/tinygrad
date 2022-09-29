import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import sys
import math
import time

from nodes.can1_node import Can1RecvItems, Can1SendItems

from missions.trackdrive import Trackdrive

from pycandb.can_interface import CanInterface

class MissionNode(mp.Process):
    def __init__(self, perception_out, can1_recv_name):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_name = can1_recv_name

        self.frequency = 10 # Hz

    def initialize(self):
        self.can1_recv_state = shared_memory.ShareableList(name=self.can1_recv_name)
        self.mission = Trackdrive(self.perception_out, self.can1_recv_state)

        self.CAN1 = CanInterface("data/D1.json", 0, False)


    def run(self):
        self.initialize()

        while True:
            start_time = time.perf_counter()

            # 1. receive perception data
            steering_angle, torq = self.mission.loop()

            self.CAN1.send_can_msg([steering_angle], self.CAN1.name2id["XVR_Control"])

            end_time = time.perf_counter()
            # print("loop_delta: ", end_time - start_time)
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)

            if time_to_sleep > 0.:
                time.sleep(time_to_sleep)