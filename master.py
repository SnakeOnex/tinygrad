import numpy as np
import pickle

import sys
import time
import os
import multiprocessing
from multiprocessing import shared_memory

from pathlib import Path
import argparse

from tvojemama.logger import gen_name_with_time, create_log_folder
from main_config import config

from autonomous.cone_detector.cone_detector import ConeDetectionNode
from missions.trackdrive.trackdrive_node import Trackdrive
from can.can1.can1_node import Can1Node, Can1RecvItems, Can1SendItems

def main(brosbag_folder=None):
    multiprocessing.set_start_method('spawn')

    # 0. create a log folder for the run
    main_log_folder = config["main_log_folder"]
    folder_path = main_log_folder / Path(gen_name_with_time('', ''))
    folder_path.mkdir()
    print("LOG_FILE: ", folder_path)

    # 1. processes init

    ## AS
    cone_detector_out = multiprocessing.Queue()
    cone_detector = ConeDetectionNode(cone_detector_out, main_log_folder, brosbag_folder)

    ## CAN
    can1_recv_state = shared_memory.ShareableList([0. for _ in range(len(Can1RecvItems))])
    can1_send_state = shared_memory.ShareableList([0. for _ in range(len(Can1SendItems))], name="can1_out")
    can1 = Can1Node(mode="SIMULATION", recv_name=can1_recv_state.shm.name, send_name=can1_send_state.shm.name)

    ## MISSIONS
    trackdrive = Trackdrive(
            perception_out=cone_detector_out,
            can1_recv_name=can1_recv_state.shm.name,
            can1_send_name=can1_send_state.shm.name
    )

    ## ASM

    # 2. start the processes
    cone_detector.start()
    time.sleep(1)
    can1.start()
    trackdrive.start()

    cone_detector.join()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--brosbag_folder', type=str, default=None)
    args = parser.parse_args()

    main(args.brosbag_folder)
