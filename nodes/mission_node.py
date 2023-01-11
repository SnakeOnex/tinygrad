import multiprocessing as mp
import pickle
import numpy as np
import zmq
import sys
import math
import time
from enum import IntEnum

from missions.acceleration import Acceleration
from missions.trackdrive import Trackdrive
from missions.skidpad import Skidpad
from missions.autocross import Autocross

from nodes.asm import ASM, AS
from config import can_config, tcp_config
from config import VisionNodeMsgPorts, CAN1NodeMsgPorts, CAN2NodeMsgPorts

from pycandb.can_interface import CanInterface

from internode_communication import create_subscriber_socket, update_subscription_data

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
    def __init__(self):
        mp.Process.__init__(self)
        self.frequency = 100  # Hz
        # Autonomous State Machine
        self.ASM = ASM()
        self.finished = False
        # Vision node data
        self.percep_data = np.zeros((0,3))
        # CAN1 node data
        self.wheel_speed = 0.
        self.mission_num = MissionValue.NoValue.value
        self.start_button = 0
        # CAN2 node data
        self.go_signal = 0

    def initialize(self):
        self.acceleration = Acceleration()
        self.autocross = Autocross()
        self.trackdrive = Trackdrive()
        self.skidpad = Skidpad()

        self.missions = [None, self.acceleration, self.skidpad, self.autocross, self.trackdrive]
        self.mission = self.missions[MissionValue.NoValue]

        self.CAN1 = CanInterface(can_config["CAN_JSON"], can_config["CAN1_ID"], False)

        self.context = zmq.Context()
        self.debug_socket = self.context.socket(zmq.PUB)
        self.debug_socket.bind(tcp_config["TCP_HOST"]+":"+tcp_config["AS_DEBUG_PORT"])

        # Vision node message subscriptions
        self.cone_preds_socket = create_subscriber_socket(VisionNodeMsgPorts.CONE_PREDS)

        # CAN1 node message subscriptions
        self.wheel_speed_socket = create_subscriber_socket(CAN1NodeMsgPorts.WHEEL_SPEED)
        self.mission_socket = create_subscriber_socket(CAN1NodeMsgPorts.MISSION)
        self.start_button_socket = create_subscriber_socket(CAN1NodeMsgPorts.START_BUTTON)

        # CAN2 node message subscriptions
        self.go_signal_socket = create_subscriber_socket(CAN2NodeMsgPorts.GO_SIGNAL)

    def run(self):
        self.initialize()

        while True:
            start_time = time.perf_counter()

            self.start_button = update_subscription_data(self.start_button_socket, self.start_button)
            self.go_signal = update_subscription_data(self.go_signal_socket, self.go_signal)

            # 1. update AS State
            # TODO: change start_button to tson_button
            self.ASM.update(start_button=self.start_button,
                            go_signal=self.go_signal,
                            finished=self.finished)

            if self.ASM.AS == AS.DRIVING:                
                self.percep_data = update_subscription_data(self.cone_preds_socket, self.percep_data)
                self.wheel_speed = update_subscription_data(self.wheel_speed_socket, self.wheel_speed)

                self.finished, steering_angle, speed, log, (path, target) = self.mission.loop(self.percep_data, self.wheel_speed)

                self.debug_socket.send(pickle.dumps({
                    "perception": self.percep_data, 
                    "path": path, 
                    "target": target, 
                    "speed": speed, 
                    "steering_angle": steering_angle, 
                    "mission_id": self.mission.ID,
                    "mission_status": log}))

                self.CAN1.send_can_msg([steering_angle], self.CAN1.name2id["XVR_Control"])
                self.CAN1.send_can_msg([0, 0, 0, 0, speed, 0], self.CAN1.name2id["XVR_SetpointsMotor_A"])
            else:
                self.mission_num = update_subscription_data(self.mission_socket, self.mission_num)

                if self.mission is None and self.start_button == 1:
                    print(f"mission: {MissionValue(self.mission_num).name}")

                self.mission = self.missions[self.mission_num]

            # 3. send XVR_STATUS
            self.CAN1.send_can_msg([self.ASM.AS.value, 0, 0, 0, 0, 0, 0, 0], self.CAN1.name2id["XVR_Status"])

            end_time = time.perf_counter()
            # print(f"loop_delta:  {(end_time - start_time)*1000}ms")
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)

            if time_to_sleep > 0.:
                time.sleep(time_to_sleep)
