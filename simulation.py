import argparse
import time
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
    parser.add_argument('--tcp', action='store_true')
    args = parser.parse_args()

    state = State(args.map)

    state.forward()

    freq = 60 # Hz
    per = 1./freq

    # visual_state = shared_memory.ShareableList([0., 0., 180., 40.], name="visual_state")
    visual_state = shared_memory.ShareableList(name="visual_state2")
    unregister("/visual_state2", "shared_memory")
    # exit(0)
    update_visual_state(visual_state, state)

    while True:
        state.update_state(per)
        state.forward()
        update_visual_state(visual_state, state)
        time.sleep(per)
