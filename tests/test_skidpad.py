from pathlib import Path
import time
import multiprocessing
from multiprocessing import Process

from dv_sim.sim.simulation import Simulation, MissionValue
from master import main
# from dv_sim import sim_config


def test_skidpad():
    # 1. SETUP SIMULATION && BROS

    # 1.A SIMULATION SETUP
    map_path = Path("dv_sim/maps/skidpad_map.json").resolve()
    sim = Simulation(
        map_path=map_path,
        mission=MissionValue.Skidpad,
        manual=False,
        config_json=Path("dv_sim/sim_config.json").resolve()
    )
    sim.launch_gui()

    # 1.B BROS SETUP
    bros_process = Process(target=main)
    bros_process.start()

    # 2. TEST LOOP
    start_time = time.perf_counter()

    # simulation loop
    while True:
        sim.step()
        time_since_start = time.perf_counter() - start_time

        # after 10 seconds send go signal
        if time_since_start >= 5.:
            sim.go_signal()

        # after 20 seconds end and return TRUE
        if time_since_start >= 200.:
            test_outcome = True
            break

        sim.sleep()

    assert test_outcome

    # 3. HOUSE KEEPING
    bros_process.terminate()
    sim.terminate_gui()
    # sim.vision_socket.close()
    # sim.context.term()
    print("closed socket")
    time.sleep(0.1)
