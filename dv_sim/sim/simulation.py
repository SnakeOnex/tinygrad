import argparse
import time
import socket
import struct
import subprocess
import os
import json
from pathlib import Path
from enum import IntEnum
import zmq
import pickle
from .state import State
import multiprocessing.connection as connection
from multiprocessing.resource_tracker import unregister
from pycandb.can_interface import CanInterface
from .state_to_can import can1_send_callbacks, can2_send_callbacks, can1_recv_callbacks
from .track_marshall import TrackMarshall

from .network_helpers import connect_client, bind_udp_socket
# from track_marshall import Track_marshall


class GUIValues(IntEnum):
    car_pos = 0,
    car_heading = 1,
    steering_angle = 2,
    car_speed = 3,
    in_start = 4,
    in_finish = 5,
    cones_mask = 6,
    go_signal = 7,
    race_time = 8,
    debug = 9


class ControlsValues(IntEnum):
    go_signal = 0,
    lat_control = 1,
    long_control = 2,
    emergency_signal = 3


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


class Simulation():
    def __init__(self, map_path, mission=MissionValue.Trackdrive, gui=False, manual=False, config_json=None):
        self.map_path = Path(map_path)
        self.manual = manual
        self.context = zmq.Context()
        # 1. setup physics state
        self.state = State(mission, self.map_path)
        self.state.manual = manual
        self.frequency = 100  # Hz
        self.period = 1. / self.frequency

        # 1.B setup trackmarshall
        self.track_marshall = TrackMarshall(self.state)

        # 1.C load config json file
        with open(config_json, 'r') as f:
            config = json.loads(f.read())

        # 2. setup communication objects

        # 2.A vision simulation address
        if not self.manual:
            self.vision_socket = self.context.socket(zmq.PUB)
            self.vision_socket.bind(config["TCP_HOST"] + ":" + config["VISION_PORT"])
            self.vision_freq = 30  # Hz
            self.vision_time = 0.  # var for keeping track of last time vision packat has been sent

        # 2.B sending gui state to the graphical engine
        self.gui_state = [0. for _ in range(len(GUIValues))]
        self.gui_socket = self.context.socket(zmq.PUB)
        self.gui_socket.bind(
            config["TCP_HOST"] + ":" + config["GUI_PORT"])

        # 2.C receiving controls commands from graphical engine
        self.controls_socket = self.context.socket(zmq.SUB)
        self.controls_socket.connect(
            config["TCP_HOST"] + ":" + config["CONTROLS_PORT"])
        self.controls_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.controls_poller = zmq.Poller()
        self.controls_poller.register(self.controls_socket, zmq.POLLIN)

        # 2.D CAN interaface objects
        self.CAN1 = CanInterface(
            config["CAN_JSON"], config["CAN1_ID"], True)
        self.CAN2 = CanInterface(
            config["CAN_JSON"], config["CAN2_ID"], True)

    def step(self):
        curr_time = time.perf_counter()

        # 1. update physical state of the world
        self.state.update_state(self.period)

        # 1.1 trackmarshall update
        self.track_marshall.update()

        # 2. handle controls from 3D engine
        self.handle_controls()

        # 3. handle sending car's vision information to the autonomous system
        if not self.manual and (curr_time - self.vision_time) >= 1. / self.vision_freq:
            self.vision_socket.send(pickle.dumps(self.state.get_detections()))
            self.vision_time = curr_time

        # 4. send CAN1 & CAN2 messages
        # CAN1 messages
        # ! DON'T FORGET TO SEND EXTENDED ID FROM MOTOR CONTROLLER
        for msg_name, callback_fn in can1_send_callbacks.items():
            values = callback_fn(self.state)
            self.CAN1.send_can_msg(values, self.CAN1.name2id[msg_name], is_extended_id=(msg_name == "MCR_ActualValues_A"))

        # CAN2 messages
        for msg_name, callback_fn in can2_send_callbacks.items():
            values = callback_fn(self.state)

            self.CAN2.send_can_msg(values, self.CAN2.name2id[msg_name])

        # 5. receive CAN1 messages
        while True:
            can_msg = self.CAN1.recv_can_msg(0.)

            if can_msg is None:
                break

            if self.CAN1.id2name[can_msg.arbitration_id] not in can1_recv_callbacks:
                continue

            # update state
            values = self.CAN1.read_can_msg(can_msg)
            can1_recv_callbacks[self.CAN1.id2name[can_msg.arbitration_id]](
                self.state, values)

        # 6. update gui state and send it to the 3D engine
        self.update_gui_state()
        self.gui_socket.send(pickle.dumps(self.gui_state))

        # print("step took: ", time.perf_counter() - curr_time)

    def sleep(self):
        time.sleep(self.period)

    def handle_controls(self):
        while self.controls_poller.poll(0.):
            controls_state = pickle.loads(self.controls_socket.recv())

            if controls_state[ControlsValues.go_signal]:
                self.go_signal()

            if controls_state[ControlsValues.emergency_signal]:
                print("emergency")
                self.emergency_signal()

            if self.manual:
                # lateral control
                if controls_state[ControlsValues.lat_control] == -1:
                    self.state.steer_left()
                elif controls_state[ControlsValues.lat_control] == 1:
                    self.state.steer_right()

                # long control
                if controls_state[ControlsValues.long_control] == -1:
                    self.state.brake()
                elif controls_state[ControlsValues.long_control] == 1:
                    self.state.forward()

    def update_gui_state(self):
        car_x, car_y = self.state.car_pos
        car_heading = self.state.heading
        steering_angle = self.state.steering_angle
        cones_mask = self.track_marshall.cones_mask

        self.gui_state[GUIValues.car_pos] = (car_x, car_y)
        self.gui_state[GUIValues.car_heading] = car_heading
        self.gui_state[GUIValues.steering_angle] = steering_angle
        self.gui_state[GUIValues.car_speed] = self.state.speed
        self.gui_state[GUIValues.cones_mask] = self.track_marshall.cones_mask
        self.gui_state[GUIValues.go_signal] = self.state.go_signal
        self.gui_state[GUIValues.race_time] = self.track_marshall.race_time
        self.gui_state[GUIValues.debug] = self.track_marshall.debug

    def go_signal(self):
        self.state.go_signal = 1

    def emergency_signal(self):
        self.state.emergency_signal = 1

    def emergency_brake(self):
        raise NotImplementedError

    def launch_gui(self):
        sim_gui_path = Path(__file__).parent.parent
        # print("gui path: ", sim_gui_path)
        cmd = f"cd {sim_gui_path} && python simulation_gui.py --map {self.map_path}&"

        cwd = os.getcwd()
        os.chdir(sim_gui_path)
        self.p = subprocess.Popen(
            ["python", "simulation_gui.py", "--map", self.map_path])
        os.chdir(cwd)

    def terminate_gui(self):
        self.p.terminate()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/circle_map.json')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--manual', action='store_true')
    args = parser.parse_args()

    sim = Simulation(map_path=args.map, gui=args.gui, manual=args.manual)

    while True:
        sim.step()
        sim.sleep()
