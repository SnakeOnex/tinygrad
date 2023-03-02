import argparse
from sim.simulation import Simulation, MissionValue
from pathlib import Path
import time

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/circle_map.json')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--manual', action='store_true')
    parser.add_argument('--mission', type=str, default=MissionValue.Trackdrive)
    parser.add_argument('--config_json', type=str, default='sim_config.json')
    args = parser.parse_args()

    if args.mission.lower() == "trackdrive":
        mission = MissionValue.Trackdrive
    elif args.mission.lower() == "autocross":
        mission = MissionValue.Autocross
    elif args.mission.lower() == "skidpad":
        mission = MissionValue.Skidpad
    elif args.mission.lower() == "acceleration":
        mission = MissionValue.Acceleration
    else:
        mission = MissionValue.Acceleration

    sim = Simulation(map_path=args.map, manual=args.manual,
                     config_json=Path('sim_config.json').resolve(),
                     mission=mission)

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
