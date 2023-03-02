from enum import Enum
import multiprocessing as mp
import subprocess

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from config import perf_logger_config as log_opt

import time
from datetime import datetime

from tvojemama.logger import Logger
from tvojemama.computer_status import get_cpu_info, get_memory_info, get_processes_info, get_gpu_info


class PerfLogNode(mp.Process):
    def __init__(self, main_log_folder, frequency=10, main_drive_path="/dev/nvme0n1p5"):
        mp.Process.__init__(self)
        self.frequency = frequency
        self.main_drive_path = main_drive_path
        self.main_log_folder = main_log_folder
        self.log_opt = log_opt
        self.logger = Logger(
            log_name=self.log_opt["log_name"], log_folder_name=self.log_opt["log_folder_name"], main_folder_path=self.main_log_folder)

        # END OF LOGGER SETUP

    def log_computer_state(self):

        cpu_info = get_cpu_info()
        mem_info = get_memory_info(self.main_drive_path)
        if self.log_gpu:
            gpu_info = get_gpu_info()
        else:
            gpu_info = "NO GPU AVAILABLE ON SYSTEM"

        # TODO implement process info for vision, slam, lidar, mission node
        # processes_info = get_processes_info([cone_detection_node, slam_node])

        log_frame_msg = {
            "cpu_info": cpu_info,
            "mem_info": mem_info,
            # "processses_info": processes_info,
            "gpu_info": gpu_info
        }

        self.logger.log("LOG_NODE_FRAME", log_frame_msg)

    def initialize(self):
        # Check NVIDIA GPU presence and set logging accordingly
        try:
            subprocess.check_output('nvidia-smi')
            print("NVIDIA GPU DETECTED ON SYSTEM")
            self.log_gpu = True
        except Exception:
            print("NO GPU DETECTED ON SYSTEM")
            self.log_gpu = False

    def run(self):
        self.initialize()
        while True:
            start_time = time.perf_counter()
            self.log_computer_state()
            end_time = time.perf_counter()
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)
            if time_to_sleep > 0.:
                time.sleep(time_to_sleep)
