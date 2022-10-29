import argparse
from simulation import Simulation

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
