import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import sys
import math
import time

from nodes.can1_node import Can1RecvItems, Can1SendItems

from missions.trackdrive import Trackdrive

class MissionNode(mp.Process):
    def __init__(self, perception_out, can1_recv_name, can1_send_name):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_name = can1_recv_name
        self.can1_send_name = can1_send_name

        self.frequency = 10 # Hz

    def initialize(self):
        self.can1_recv_state = shared_memory.ShareableList(name=self.can1_recv_name)
        self.can1_send_state = shared_memory.ShareableList(name=self.can1_send_name)
        self.mission = Trackdrive(self.perception_out, self.can1_recv_state, self.can1_send_state)

    def run(self):
        self.initialize()

        while True:
            start_time = time.perf_counter()
            print("start: ", start_time)

            # 1. receive perception data
            self.mission.loop()

            end_time = time.perf_counter()
            print("end: ", end_time)
            print("loop_delta: ", end_time - start_time)
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)

            if time_to_sleep > 0.:
                print("sleeping: ", time_to_sleep)
                time.sleep(time_to_sleep)
            
