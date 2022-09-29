import argparse
import time
import socket
import struct

from multiprocessing import shared_memory
from state import State
import multiprocessing.connection as connection
from multiprocessing.resource_tracker import unregister
from pycandb.can_interface import CanInterface
from state_to_can import can1_send_callbacks, can1_recv_callbacks

def update_visual_state(visual_state, state):
    car_x, car_y = state.car_pos
    car_heading = state.heading
    steering_angle = state.steering_angle

    visual_state[0] = car_x
    visual_state[1] = car_y
    visual_state[2] = car_heading
    visual_state[3] = steering_angle

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/circle_map.json')
    parser.add_argument('--comm', type=str, default='udp') # shared_mem
    args = parser.parse_args()

    state = State(args.map)

    # vision simulation connection
    remote_address = "localhost", 50000
    listener = connection.Listener(remote_address)
    vision_conn = listener.accept()
    vision_freq = 30 # hz
    vision_time = 0.

    ## visual state communication with graphical interface setup
    visual_state = [0., 0., 0., 0.]
    if args.comm == "shared_mem":
        # visual_state = shared_memory.ShareableList(visual_state, name="visual_state")
        visual_state = shared_memory.ShareableList(name="visual_state")
        unregister("/visual_state", "shared_memory")
    update_visual_state(visual_state, state)

    ## Visual state connection 
    visual_addr = ('127.0.0.1', 1337)
    visual_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    map_addr = ('127.0.0.1', 1338)
    map_socket = socket.socket(socket.AF_INET, SOCK_DGRAM)

    ## CAN interface setup
    CAN1 = CanInterface("can_lib/D1.json", 0, True)

    ## simulation frequency
    freq = 100 # Hz
    per = 1./freq

    while True:
        state.update_state(per)
        state.forward()
        # state.steer_left()

        curr_time = time.perf_counter()

        # send vision simulation
        if (curr_time - vision_time) >= 1. / vision_freq:
            # print("sending detecitons")
            vision_conn.send(state.get_detections())
            vision_time = curr_time

        # send CAN1 messages
        for msg_name, callback_fn in can1_send_callbacks.items():
            values = callback_fn(state)

            CAN1.send_can_msg(values, CAN1.name2id[msg_name])

        # receive CAN1 messages
        while True:
            can_msg = CAN1.recv_can_msg(0.)
            
            if can_msg is None or CAN1.id2name[can_msg.arbitration_id] not in can1_recv_callbacks:
                break

            # update state
            values = CAN1.read_can_msg(can_msg)
            can1_recv_callbacks[CAN1.id2name[can_msg.arbitration_id]](state, values)

        # update visual state and send to simulation graphical visualizer
        update_visual_state(visual_state, state)
        if args.comm == "udp":
            data = struct.pack('<4f', *visual_state)
            visual_socket.sendto(data, client)
        elif args.comm == "shared_mem":
            # nothing needs to be done
            pass

        # exit(0)
        time.sleep(per)
