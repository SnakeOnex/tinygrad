import multiprocessing as mp
import subprocess
from config import perf_logger_config as log_opt
import time
from tvojemama.logger import Logger
from tvojemama.computer_status import get_cpu_info, get_memory_info, get_ssd_info, get_process_info, get_gpu_info


class PerfLogNode(mp.Process):
    def __init__(self, main_log_folder, frequency=10):
        mp.Process.__init__(self)
        self.frequency = frequency
        self.main_log_folder = main_log_folder

        # END OF PERFLOGGER SETUP

    def log_computer_state(self):
        # TODO implement process info for vision, slam, lidar, mission node

        log_frame_msg = {
            "cpu_info": get_cpu_info(),
            "mem_info": get_memory_info(),
            "ssd_info": get_ssd_info(),
            "gpu_info": get_gpu_info() if self.log_gpu else "NO GPU ON SYSTEM",
            # TODO "processses_info": processes_info,
        }
        # print(log_frame_msg)
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

    def run(self):
        self.initialize()
        while True:
            start_time = time.perf_counter()
            self.log_computer_state()
            end_time = time.perf_counter()
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)
            if time_to_sleep > 0.:
                time.sleep(time_to_sleep)
