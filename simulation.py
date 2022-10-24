import argparse
import time
import socket
import struct
import os
# from simulation_gui import KEYBOARDS_PORT

from state import State
import multiprocessing.connection as connection
from multiprocessing.resource_tracker import unregister
from pycandb.can_interface import CanInterface
from state_to_can import can1_send_callbacks, can2_send_callbacks, can1_recv_callbacks

from network_helpers import connect_client, bind_udp_socket
from track_marshall import Track_marshall

HOST = '127.0.0.1'
VISUAL_PORT = 1337
CONTROLS_PORT = 1338
KEYBOARDS_PORT = 1339


def update_visual_state(visual_state, state):
    car_x, car_y = state.car_pos
    car_heading = state.heading
    steering_angle = state.steering_angle

    visual_state[0] = car_x
    visual_state[1] = car_y
    visual_state[2] = car_heading
    visual_state[3] = steering_angle


def handle_keys(keyboard_poller, keyboard_socket, state):
    while True:
        evts = keyboard_poller.poll(0.)
        if len(evts) == 0:
            break
        sock, evt = evts[0]
        if evt:
            data = keyboard_socket.recvfrom(16)
            keyboard_state = struct.unpack('<4i', data[0])

            if keyboard_state[0]:
                state.forward()
            elif keyboard_state[1]:
                state.brake()
            if keyboard_state[2]:
                state.steer_left()
            elif keyboard_state[3]:
                state.steer_right()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/circle_map.json')
    parser.add_argument('--comm', type=str, default='udp')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--autonomous', action='store_true')
    args = parser.parse_args()

    if args.gui:
        os.system("python simulation_gui.py&")

    state = State(args.map)

    # vision simulation connection
    remote_address = "localhost", 50000
    if args.autonomous:
        listener = connection.Listener(remote_address)
        vision_conn = listener.accept()
    vision_freq = 30 # hz
    vision_time = 0.

    ## visual state communication with graphical interface setup
    visual_state = [0., 0., 0., 0.]

    update_visual_state(visual_state, state)

    ## Visual state connection

    visual_addr = (HOST, VISUAL_PORT)
    visual_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    controls_socket, controls_poller = bind_udp_socket(HOST, CONTROLS_PORT)
    keyboard_socket, keyboard_poller = bind_udp_socket(HOST, KEYBOARDS_PORT)

    ## CAN interface setup
    CAN1 = CanInterface("data/D1.json", 0, True)
    CAN2 = CanInterface("data/D1.json", 1, True)

    ## simulation frequency
    freq = 100 # Hz
    per = 1./freq

    while True:
        state.update_state(per)
        # state.forward()
        # state.steer_left()

        curr_time = time.perf_counter()

        # receive controls
        while True:
            evts = controls_poller.poll(0.)
            if len(evts) == 0:
                break
            sock, evt = evts[0]
            if evt:
                data = controls_socket.recvfrom(16)
                controls_state = struct.unpack('<4i', data[0])

                if controls_state[0]:
                    print("SENDING GO SIGNAL")
                    track_marshall = Track_marshall(state)
                    state.go_signal = 1

        # send vision simulation
        if args.autonomous and (curr_time - vision_time) >= 1. / vision_freq:
            # print("sending detecitons")

            vision_conn.send(state.get_detections())
            vision_time = curr_time
        elif not args.autonomous:
            handle_keys(keyboard_poller, keyboard_socket, state)
        # send CAN1 messages
        for msg_name, callback_fn in can1_send_callbacks.items():
            values = callback_fn(state)
            CAN1.send_can_msg(values, CAN1.name2id[msg_name])

        # # send CAN2 messages
        for msg_name, callback_fn in can2_send_callbacks.items():
            values = callback_fn(state)

            # print(f"sending {msg_name} message, values: {values}")
            CAN2.send_can_msg(values, CAN2.name2id[msg_name])

        # # receive CAN1 messages
        while True:
            can_msg = CAN1.recv_can_msg(0.)

            if can_msg is None:
                break

            if CAN1.id2name[can_msg.arbitration_id] not in can1_recv_callbacks:
                continue

            # update state
            values = CAN1.read_can_msg(can_msg)
            can1_recv_callbacks[CAN1.id2name[can_msg.arbitration_id]](state, values)

        if not args.autonomous:
            handle_keys(keyboard_poller, keyboard_socket, state)

        # update visual state and send to simulation graphical visualizer
        update_visual_state(visual_state, state)
        if args.comm == "udp":
            data = struct.pack('<4f', *visual_state)
            visual_socket.sendto(data, visual_addr)

        # exit(0)
        time.sleep(per)