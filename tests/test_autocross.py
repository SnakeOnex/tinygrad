from pathlib import Path
import time
import os
import unittest
import multiprocessing
from multiprocessing import Process

import sys
# the mock-0.3.1 dir contains testcase.py, testutils.py & mock.py
sys.path.append('..')

import dv_sim
from dv_sim.sim.simulation import Simulation, MissionValue
from nodes.asm import AS
from master import main
# from dv_sim import sim_config

class TestAutocross(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # 1. SETUP SIMULATION && BROS

        # 1.A SIMULATION SETUP
        root_folder_path = Path(__file__).parent.parent
        os.chdir(root_folder_path)
        map_path = Path("dv_sim/maps/circle_map.json").resolve()
        print(map_path)
        # exit(0)
        cls.sim = Simulation(
                map_path=map_path,
                mission=MissionValue.Autocross,
                manual=False,
                config_json=Path("dv_sim/sim_config.json").resolve()
        )
        cls.sim.launch_gui()

        # 1.B BROS SETUP
        cls.bros_process = Process(target=main)
        cls.bros_process.start()

    @classmethod
    def tearDownClass(cls):
        print("tearing down!")
        cls.bros_process.terminate()
        cls.sim.terminate_gui()

        del cls.bros_process
        del cls.sim

        time.sleep(1)

    def test_autocross(self):
        # 2. TEST LOOP
        start_time = time.perf_counter()

        # simulation loop
        while True:
            self.sim.step()
            time_since_start = time.perf_counter() - start_time

            # after 10 seconds send go signal
            if time_since_start >= 5.:
                self.sim.go_signal()

            # after 20 seconds end and return TRUE
            if time_since_start >= 31.:
                test_outcome = True
                break
                
            # if state is finished end simulation
            if self.sim.state.AS == AS.FINISHED:
                test_outcome = True
                break

            self.sim.sleep()

        assert test_outcome



# def test_autocross():
#     # 1. SETUP SIMULATION && BROS

#     # 1.A SIMULATION SETUP
#     root_folder_path = Path(__file__).parent.parent
#     os.chdir(root_folder_path)
#     map_path = Path("dv_sim/maps/circle_map.json").resolve()
#     print(map_path)
#     # exit(0)
#     sim = Simulation(
#             map_path=map_path,
#             mission=MissionValue.Autocross,
#             manual=False,
#             config_json=Path("dv_sim/sim_config.json").resolve()
#     )
#     sim.launch_gui()

#     # 1.B BROS SETUP
#     bros_process = Process(target=main)
#     bros_process.start()

#     # 2. TEST LOOP
#     start_time = time.perf_counter()

#     # simulation loop
#     while True:
#         sim.step()
#         time_since_start = time.perf_counter() - start_time

#         # after 10 seconds send go signal
#         if time_since_start >= 5.:
#             sim.go_signal()

#         # after 20 seconds end and return TRUE
#         if time_since_start >= 31.:
#             test_outcome = True
#             break
            
#         # if state is finished end simulation
#         if sim.state.AS == AS.FINISHED:
#             test_outcome = True
#             break

#         sim.sleep()

#     assert test_outcome

#     # 3. HOUSE KEEPING
#     bros_process.terminate()
#     sim.terminate_gui()
#     time.sleep(0.1)

if __name__ == "__main__":
    exit()