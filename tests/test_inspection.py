from pathlib import Path
import time
import os
import unittest
from multiprocessing import Process

import sys
sys.path.append('.')

from dv_sim.sim.simulation import Simulation, MissionValue
from nodes.asm import AS
from master import main
from config import inspection_config


class TestInspection(unittest.TestCase):

    def setUp(cls):
        # 1. SETUP SIMULATION && BROS

        # 1.A SIMULATION SETUP

        map_path = Path("dv_sim/maps/blank_map.json").resolve()
        cls.sim = Simulation(
            map_path=map_path,
            mission=MissionValue.Inspection,
            manual=False,
            config_json=Path("dv_sim/sim_config.json").resolve()
        )
        cls.sim.launch_gui()

        # 1.B BROS SETUP
        cls.bros_process = Process(target=main, args=["SIM"])
        cls.bros_process.start()
        time.sleep(1)

    def tearDown(cls):
        print("tearing down!")
        cls.bros_process.terminate()
        cls.sim.terminate_gui()

        del cls.bros_process
        del cls.sim
        time.sleep(1)

    def test_inspection(self):
        start_time = time.perf_counter()

        # simulation loop
        while True:
            self.sim.step()
            time_since_start = time.perf_counter() - start_time

            # after 10 seconds send go signal
            if time_since_start >= 5.:
                self.sim.go_signal()

            # after max seconds end and return TRUE
            if time_since_start >= inspection_config["duration"] + 6.:
                test_outcome = False
                break

            # if state is finished end simulation
            if self.sim.state.AS == AS.FINISHED:
                test_outcome = True
                break

            self.sim.sleep()

        assert test_outcome


if __name__ == "__main__":
    unittest.main()
