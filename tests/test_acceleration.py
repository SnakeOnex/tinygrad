from pathlib import Path
import time
import multiprocessing
from multiprocessing import Process

from dv_sim.sim.simulation import Simulation
from master import main

def test_acceleration():
    # 1. SETUP
    def run_sim(ret):
        map_path = Path("dv_sim/maps/acceleration_map.json").resolve()
        print("map_path: ", map_path)
        sim = Simulation(
                map_path=map_path,
                gui=True,
                manual=False
        )
        print("started simulation")

        start_time = time.perf_counter()

        # simulation loop
        while True:
            sim.step()
            time_since_start = time.perf_counter() - start_time

            # after 10 seconds send go signal
            if time_since_start >= 10.:
                sim.go_signal()

            # after 20 seconds end and return TRUE
            if time_since_start >= 30.:
                ret.value = True
                break

            sim.sleep()

    ret = multiprocessing.Value('b', False)
    sim_process = Process(target=run_sim, args=(ret,))
    sim_process.start()

    bros_process = Process(target=main)
    bros_process.start()

    sim_process.join()

    assert ret.value

if __name__ == '__main__':
    test_acceleration()
