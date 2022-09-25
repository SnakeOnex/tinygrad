import argparse
import time
import socket
import struct

from multiprocessing import shared_memory
from state import State
from multiprocessing.resource_tracker import unregister


def update_visual_state(visual_state, state):
    car_x, car_y = state.car_pos
    car_heading = state.heading
    steering_angle = state.steering_angle

    visual_state[0] = int(car_x)
    visual_state[1] = int(car_y)
    visual_state[2] = int(car_heading)
    visual_state[3] = int(steering_angle)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/circle_map.json')
    parser.add_argument('--comm', type=str, default='udp') # shared_mem
    args = parser.parse_args()

    state = State(args.map)

    state.forward()

    freq = 1000 # Hz
    per = 1./freq

    visual_state = [0., 0., 0., 0.]

    if args.comm == "shared_mem":
        # visual_state = shared_memory.ShareableList(visual_state, name="visual_state")
        visual_state = shared_memory.ShareableList(name="visual_state")
        unregister("/visual_state", "shared_memory")

    update_visual_state(visual_state, state)

    client = ('127.0.0.1', 1337)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    data = struct.pack('<4f', *visual_state)
    print("data: ", data)
    print("len: ", len(data))
    s.sendto(data, client)

    # exit(0)

    while True:
        state.update_state(per)
        state.forward()

        # update visual state and send to simulation graphical visualizer
        update_visual_state(visual_state, state)

        if args.comm == "udp":
            data = struct.pack('<4f', *visual_state)
            s.sendto(data, client)
        elif args.comm == "shared_mem":
            # nothing needs to be done
            pass

        time.sleep(per)
