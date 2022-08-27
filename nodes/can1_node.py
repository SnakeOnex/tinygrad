import multiprocessing as mp
import multiprocessing.connection as connection
from multiprocessing import shared_memory
from enum import Enum
import time

class Can1RecvItems(Enum):
    wheel_speed = 0
    steering_actual = 1

class Can1SendItems(Enum):
    steering = 0
    engine_force = 1

class Can1Node(mp.Process):
    def __init__(self, mode, recv_name, send_name):
        mp.Process.__init__(self)
        self.mode = mode # "SIMULATION", "CAN"
        self.listen_address = "localhost", 50001
        self.send_address = "localhost", 50002
        self.recv_name = recv_name
        self.send_name = send_name

        self.report_rate = 100 # hz

    def initialize(self):
        print("CAN1 INIT")
        self.can1_recv_state = shared_memory.ShareableList(name=self.recv_name)
        self.can1_send_state = shared_memory.ShareableList(name=self.send_name)

    def run(self):
        self.initialize()

        if self.mode == "SIMULATION":
            """
            Listen for messages about the state of the simulation and put them into shared memory
            """

            print("CONNECTING")
            conn_recv = connection.Client(self.listen_address)
            print("CAN1 RECEIVE CONNECTED")
            time.sleep(0.1)
            conn_send = connection.Client(self.send_address)
            print("CAN1 SEND CONNECTED")

            while True:
                if conn_recv.poll():
                    can1_recv_state_list = conn_recv.recv()

                    for i in range(len(Can1RecvItems)):
                        self.can1_recv_state[i] = can1_recv_state_list[i]


                conn_send.send(self.can1_send_state)
                time.sleep(1. / self.report_rate)
