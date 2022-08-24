import numpy as np
import pickle

import sys
import time
import os
import multiprocessing
import torch.multiprocessing as mp

from pathlib import Path
import argparse

from tvojemama.logger import gen_name_with_time, create_log_folder
from main_config import config

from autonomous.cone_detector.cone_detector import ConeDetectionNode
from missions.trackdrive.trackdrive_node import Trackdrive

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

    ## MISSIONS
    trackdrive = Trackdrive(perception_out=cone_detector_out)

    ## CAN

    ## ASM

    # 2. start the processes
    cone_detector.start()
    trackdrive.start()

    cone_detector.join()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--brosbag_folder', type=str, default=None)
    args = parser.parse_args()

    main(args.brosbag_folder)
