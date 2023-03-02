import numpy as np
import pickle

import sys
import time
import os
import multiprocessing
from signal import signal
from signal import SIGTERM

from pathlib import Path
import argparse

from tvojemama.logger import gen_name_with_time, create_log_folder
from config import config

from nodes.vision_node import VisionNode
from nodes.can1_node import Can1Node
from nodes.can2_node import Can2Node
from nodes.mission_node import MissionNode
from nodes.log_node import PerfLogNode


def main(brosbag_folder=None):
    # multiprocessing.set_start_method('spawn')

    # 0. create a log folder for the run
    main_log_folder = config["main_log_folder"]
    folder_path = main_log_folder / Path(gen_name_with_time('', ''))
    folder_path.mkdir(parents=True)
    print("LOG_FILE: ", folder_path)

    # 1. processes init

    # AS
    vision_node = VisionNode(main_log_folder, brosbag_folder)

    # CAN
    can1_node = Can1Node(mode="SIM")
    can2_node = Can2Node(mode="SIM")

    # MISSIONS
    mission_node = MissionNode(mode="SIM")

    # ASM

    # LOG
    perf_log_node = PerfLogNode(main_log_folder=main_log_folder, mode="SIM")

    # 2. start the processes
    vision_node.start()
    print("VISION NODE STARTED")
    # time.sleep(4)
    can1_node.start()
    print("CAN1 NODE STARTED")
    can2_node.start()
    print("CAN2 NODE STARTED")
    mission_node.start()
    print("MISSION NODE STARTED")
    perf_log_node.start()
    print("PERF LOG NODE STARTED")

    def handler(sig, frame):
        vision_node.terminate()
        can1_node.terminate()
        can2_node.terminate()
        mission_node.terminate()
        perf_log_node.terminate()
        print("BROS: TERMINATED ALL CHILD PROCESSES")
        sys.exit(0)

    signal(SIGTERM, handler)

    vision_node.join()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--brosbag_folder', type=str, default=None)
    parser.add_argument('--tcp', type=bool, default=False)
    args = parser.parse_args()

    main(args.brosbag_folder)
