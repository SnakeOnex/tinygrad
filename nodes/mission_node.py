import multiprocessing as mp
from multiprocessing import shared_memory
import pickle
import numpy as np
import zmq
import sys
import math
import time
from enum import IntEnum

from nodes.can1_node import Can1RecvItems, Can1SendItems
from nodes.can2_node import Can2RecvItems, Can2SendItems

from missions.acceleration import Acceleration
from missions.trackdrive import Trackdrive
from missions.skidpad import Skidpad
from missions.autocross import Autocross

from nodes.asm import ASM, AS
from config import can_config
from config import tcp_config

from pycandb.can_interface import CanInterface


class MissionValue(IntEnum):
    NoValue = 0,
    Acceleration = 1,
    Skidpad = 2,
    Autocross = 3,
    Trackdrive = 4,
    EBS_Test = 5,
    Inspection = 6,
    Manual = 7,
    Disco = 8,
    Donuts = 9


class MissionNode(mp.Process):
    def __init__(self, perception_out, can1_recv_name, can2_recv_name):
        mp.Process.__init__(self)
        self.perception_out = perception_out
        self.can1_recv_name = can1_recv_name
        self.can2_recv_name = can2_recv_name

        self.frequency = 10  # Hz

        # Autonomous State Machine
        self.ASM = ASM()

    def initialize(self):
        self.can1_recv_state = shared_memory.ShareableList(
            name=self.can1_recv_name)
        self.can2_recv_state = shared_memory.ShareableList(
            name=self.can2_recv_name)

        self.acceleration = Acceleration(
            self.perception_out, self.can1_recv_state)
        self.autocross = Autocross(self.perception_out, self.can1_recv_state)
        self.trackdrive = Trackdrive(self.perception_out, self.can1_recv_state)
        self.skidpad = Skidpad(self.perception_out, self.can1_recv_state)

        self.missions = [None, self.acceleration,
                         self.autocross, self.trackdrive, self.skidpad]
        self.mission = self.missions[MissionValue.NoValue]

        self.CAN1 = CanInterface(
            can_config["CAN_JSON"], can_config["CAN1_ID"], False)

        self.context = zmq.Context()
        self.debug_socket = self.context.socket(zmq.PUB)
        self.debug_socket.bind(
            tcp_config["TCP_HOST"]+":"+tcp_config["AS_DEBUG_PORT"])

    def run(self):
        self.initialize()

        while True:
            start_time = time.perf_counter()

            # 1. update AS State

            # TODO: change start_button to tson_button
            self.ASM.update(start_button=self.can1_recv_state[Can1RecvItems.start_button.value],
                            go_signal=self.can2_recv_state[Can2RecvItems.go_signal.value])

            if self.ASM.AS == AS.DRIVING:
                percep_data = self.perception_out.get()
                while not self.perception_out.empty():
                    percep_data = self.perception_out.get()

                steering_angle, speed = self.mission.loop(percep_data)

                self.debug_socket.send(pickle.dumps(percep_data))

                self.CAN1.send_can_msg(
                    [steering_angle], self.CAN1.name2id["XVR_Control"])
                self.CAN1.send_can_msg(
                    [0, 0, 0, 0, speed, 0], self.CAN1.name2id["XVR_SetpointsMotor_A"])
            else:

                if self.mission is None and self.can1_recv_state[int(Can1RecvItems.start_button.value)] == 1:
                    print(
                        f"mission: {MissionValue(self.can1_recv_state[Can1RecvItems.mission.value]).name}")

                self.mission = self.missions[int(
                    self.can1_recv_state[Can1RecvItems.mission.value])]

            # 3. send XVR_STATUS
            self.CAN1.send_can_msg(
                [self.ASM.AS.value, 0, 0, 0, 0, 0, 0, 0], self.CAN1.name2id["XVR_Status"])

            end_time = time.perf_counter()
            # print(f"loop_delta:  {(end_time - start_time)*1000}ms")
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)

            if time_to_sleep > 0.:
                time.sleep(time_to_sleep)
