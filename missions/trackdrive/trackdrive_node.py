import multiprocessing as mp
import numpy as np
import sys

class Trackdrive(mp.Process):
    def __init__(self, perception_out, can1_out=None, can2_out=None):
        mp.Process.__init__(self)
        self.perception_out = perception_out

    def run(self):
        while True:
            # 1. receive perception data
            percep_data = self.perception_out.get()

            # print(percep_data)

    def receive_perception_data(self, cone_data):

        # 1. parse data

        # 2. run lateral and longitudinal controller

        # 3. send data to can1
        pass

    def receive_can1_data(self, can1_data):
        pass

    def receive_can2_data(self, can2_data):
        pass


