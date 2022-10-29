from pathlib import Path
import time
from multiprocessing import Process

from dv_sim.sim.simulation import Simulation
from master import main

def test_acceleration():
    # 1. SETUP
    def run_sim():
        map_path = Path("dv_sim/maps/acceleration_map.json").resolve()
        print("map_path: ", map_path)
        sim = Simulation(
                map_path=map_path,
                gui=True,
                manual=False
        )
        print("started simulation")

        start_time = time.perf_counter()

        while True:
            sim.step()

            time_since_start = time.perf_counter() - start_time

            if time_since_start >= 5.:
                sim.go_signal()

            sim.sleep()

    sim_process = Process(target=run_sim)
    sim_process.start()

    bros_process = Process(target=main)
    bros_process.start()

    sim_process.join()

    assert True

if __name__ == '__main__':
    test_acceleration()
