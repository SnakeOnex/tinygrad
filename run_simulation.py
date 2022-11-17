import argparse
from sim.simulation import Simulation
import time
import sim_config

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/circle_map.json')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--manual', action='store_true')
    args = parser.parse_args()

    sim = Simulation(map_path=args.map, manual=args.manual,
                     tcp_config=sim_config.tcp_config, can_config=sim_config.can_config)

    if args.gui:
        sim.launch_gui()

    start_time = time.perf_counter()
    while True:
        sim.step()
        sim.sleep()

        # if time.perf_counter() - start_time > 5:
        # print("5 secs passed")
        # sim.terminate_gui()
        # break

    # print("ended")
