import numpy as np
import pickle

import sys
import time
import os
import multiprocessing
from pathlib import Path
import argparse

from autonomous.cone_detector.cone_detector import ConeDetectionNode
from tvojemama.logger import gen_name_with_time, create_log_folder
from main_config import config


def main(brosbag_folder=None):

    # 0. create a log folder for the run
    main_log_folder = config["main_log_folder"]
    folder_path = main_log_folder / Path(gen_name_with_time('', ''))
    folder_path.mkdir()
    print("LOG_FILE: ", folder_path)

    # 1. processes init

    ## AS
    cone_to_path = multiprocessing.Queue()
    cone_detector = ConeDetectionNode(cone_to_path, main_log_folder, brosbag_folder)

    path_to_mission = multiprocessing.Queue()
    path_planning = PathPlanning(input_queue=cone_to_path, path_to_mission, main_log_folder)

    ## MISSIONS
    # trackdrive = Trackdrive()

    ## CAN

    ## ASM

    # 2. start the processes
    cone_detector.start()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--brosbag_folder', type=str, default=None)
    args = parser.parse_args()

    main(args.brosbag_folder)
