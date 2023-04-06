import multiprocessing as mp
import subprocess
from config import perf_logger_config as log_opt
import time
from tvojemama.logger import Logger
from tvojemama.computer_status import *


class PerfLogNode(mp.Process):
    def __init__(self, main_log_folder, process_pids, frequency=10):
        mp.Process.__init__(self)
        self.frequency = frequency
        self.main_log_folder = main_log_folder
        self.process_pids = process_pids
        self.gpu_pids = {key: value for key, value in process_pids.items() if value in log_opt["gpu_processes"]}

        # END OF PERFLOGGER SETUP

    def log_computer_state(self):

        log_frame_msg = {
            "cpu_info": get_cpu_info(),
            "mem_info": get_memory_info(),
            "ssd_info": get_ssd_info(),
            "gpu_info": get_gpu_info() if self.log_gpu else "NO GPU ON SYSTEM",
            "process_stats": get_cpu_mem_process_info(self.process_pids),
            "gpu_process_stats": get_gpu_process_info(self.gpu_pids) if self.log_gpu else "NO GPU ON SYSTEM"
        }

        self.logger.log("LOG_NODE_FRAME", log_frame_msg)

    def initialize(self):
        self.log_opt = log_opt
        self.logger = Logger(
            log_name=self.log_opt["log_name"],
            log_folder_name=self.log_opt["log_folder_name"],
            main_folder_path=self.main_log_folder
        )

        # Check NVIDIA GPU presence and set logging accordingly
        try:
            subprocess.check_output('nvidia-smi')
            print("NVIDIA GPU DETECTED ON SYSTEM")
            self.log_gpu = True
        except Exception:
            print("NO GPU DETECTED ON SYSTEM")
            self.log_gpu = False

        # Add self to the tracked processes
        self.process_pids[str(self.pid)] = "PERF_LOGGER_NODE"

    def run(self):
        self.initialize()
        while True:
            start_time = time.perf_counter()
            self.log_computer_state()
            end_time = time.perf_counter()
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)
            if time_to_sleep > 0.:
                time.sleep(time_to_sleep)
