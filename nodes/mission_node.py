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

from nodes.node_msgs import create_subscriber_socket, get_last_subscription_data
from nodes.node_msgs import VisionNodeMsgPorts

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
    def __init__(self, can1_recv_name, can2_recv_name):
        mp.Process.__init__(self)
        self.can1_recv_name = can1_recv_name
        self.can2_recv_name = can2_recv_name
        self.frequency = 100  # Hz
        # Autonomous State Machine
        self.ASM = ASM()
        self.finished = False

        self.percep_data = np.zeros((0,3))

    def initialize(self):
        self.can1_recv_state = shared_memory.ShareableList(
            name=self.can1_recv_name)
        self.can2_recv_state = shared_memory.ShareableList(
            name=self.can2_recv_name)

        self.acceleration = Acceleration(self.can1_recv_state)
        self.autocross = Autocross(self.can1_recv_state)
        self.trackdrive = Trackdrive(self.can1_recv_state)
        self.skidpad = Skidpad(self.can1_recv_state)

        self.missions = [None, self.acceleration, self.skidpad, self.autocross, self.trackdrive]
        self.mission = self.missions[MissionValue.NoValue]

        self.CAN1 = CanInterface(can_config["CAN_JSON"], can_config["CAN1_ID"], False)

        self.context = zmq.Context()
        self.debug_socket = self.context.socket(zmq.PUB)
        self.debug_socket.bind(tcp_config["TCP_HOST"]+":"+tcp_config["AS_DEBUG_PORT"])

        self.cone_preds_socket = create_subscriber_socket(VisionNodeMsgPorts.CONE_PREDS)

    def run(self):
        self.initialize()

        while True:
            start_time = time.perf_counter()

            # 1. update AS State
            # TODO: change start_button to tson_button
            self.ASM.update(start_button=self.can1_recv_state[Can1RecvItems.start_button.value],
                            go_signal=self.can2_recv_state[Can2RecvItems.go_signal.value],
                            finished=self.finished)

            if self.ASM.AS == AS.DRIVING:                
                data = get_last_subscription_data(self.cone_preds_socket)
                if data != None:
                    self.percep_data = pickle.loads(data)

                self.finished, steering_angle, speed, log, path = self.mission.loop(self.percep_data)

                self.debug_socket.send(pickle.dumps({
                    "perception": self.percep_data, 
                    "path": path, 
                    "speed": speed, 
                    "steering_angle": steering_angle, 
                    "mission_id": self.mission.ID,
                    "mission_status": log}))

                self.CAN1.send_can_msg([steering_angle], self.CAN1.name2id["XVR_Control"])
                self.CAN1.send_can_msg([0, 0, 0, 0, speed, 0], self.CAN1.name2id["XVR_SetpointsMotor_A"])
            else:
                if self.mission is None and self.can1_recv_state[int(Can1RecvItems.start_button.value)] == 1:
                    print(f"mission: {MissionValue(self.can1_recv_state[Can1RecvItems.mission.value]).name}")

                self.mission = self.missions[int(self.can1_recv_state[Can1RecvItems.mission.value])]

            # 3. send XVR_STATUS
            self.CAN1.send_can_msg([self.ASM.AS.value, 0, 0, 0, 0, 0, 0, 0], self.CAN1.name2id["XVR_Status"])

            end_time = time.perf_counter()
            # print(f"loop_delta:  {(end_time - start_time)*1000}ms")
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)

            if time_to_sleep > 0.:
                time.sleep(time_to_sleep)
