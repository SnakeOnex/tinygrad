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
from nodes.can_sender_node import CanSenderNode
from nodes.perf_log_node import PerfLogNode


def main(mode="RACE"):
    # 0. create a log folder for the run
    main_log_folder = config["main_log_folder"]
    curr_log_folder = main_log_folder / Path(gen_name_with_time('', ''))
    curr_log_folder.mkdir(parents=True)
    curr_log_folder = str(curr_log_folder)
    print("LOG_FILE: ", curr_log_folder)

    # 1. processes init

    # AS
    vision_node = VisionNode(curr_log_folder=curr_log_folder, mode=mode)

    # CAN
    can1_node = Can1Node(curr_log_folder=curr_log_folder)
    can2_node = Can2Node(curr_log_folder=curr_log_folder)

    # MISSIONS
    mission_node = MissionNode(curr_log_folder=curr_log_folder, mode=mode)
    can_sender_node = CanSenderNode(curr_log_folder=curr_log_folder)

    # ASM

    # 2. start the processes
    vision_node.start()
    print("VISION NODE STARTED")
    can1_node.start()
    print("CAN1 NODE STARTED")
    can2_node.start()
    print("CAN2 NODE STARTED")
    mission_node.start()
    print("MISSION NODE STARTED")
    can_sender_node.start()
    print("CAN SENDER NODE STARTED")

    # START LOGGER NODE WITH GIVEN PIDS TO MONITOR ALL BROS PROCESSES
    process_pids = {
        str(vision_node.pid): 'VISION_NODE',
        str(can1_node.pid): 'CAN1_NODE',
        str(can2_node.pid): 'CAN2_NODE',
        str(mission_node.pid): 'MISSION_NODE',
        str(can_sender_node.pid): 'CAN_SENDER'
    }

    perf_log_node = PerfLogNode(curr_log_folder=curr_log_folder, process_pids=process_pids)
    perf_log_node.start()
    print("PERF LOG NODE STARTED")
    time.sleep(1)
    nodes = [vision_node, can1_node, can2_node, mission_node, can_sender_node, perf_log_node]

    def handler(sig, frame):
        vision_node.terminate()
        can1_node.terminate()
        can2_node.terminate()
        mission_node.terminate()
        can_sender_node.terminate()
        perf_log_node.terminate()
        print("BROS: TERMINATED ALL CHILD PROCESSES")
        sys.exit(0)

    signal(SIGTERM, handler)

    for node in nodes:
        try:
            node.join()
        except KeyboardInterrupt:
            handler(None, None)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default="RACE")
    args = parser.parse_args()

    main(args.mode)
