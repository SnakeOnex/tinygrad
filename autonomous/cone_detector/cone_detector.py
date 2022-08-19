import multiprocessing as mp
import multiprocessing.connection as connection
# import torch.multiprocessing as mp
# import pathos.multiprocessing as mp
import time
import sys

from .cone_detector_config import config
from cones.cone_detector import ConeDetector
from cones.cone_localizer import ConeLocalizer
from .path_planning import PathPlanner
from tvojemama.logger import Logger, LogReader, name_to_log

# class ConeDetectionNode(multiprocessing.Process):
class ConeDetectionNode(mp.Process):
    def __init__(self, output_queue, main_log_folder, brosbag_path=None):
        # multiprocessing.Process.__init__(self)
        mp.Process.__init__(self)
        self.output_queue = output_queue
        self.brosbag_path = brosbag_path
        self.main_log_folder = main_log_folder
        self.log_opt = config["logging_opt"]

        self.mode = "SIMULATION" # "BROSBAG", "CAMERA", "SIMULATION"

    def initialize(self):
        print("INITTING")
        # zed setup or brosbag setup
        if self.mode == "CAMERA":
            import pyzed.sl as sl
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps = 60
            self.zed_image = sl.Mat()
            self.runtime_parameters = sl.RuntimeParameters()
        elif self.mode == "BROSBAG":
            self.brosbag_gen = LogReader(name_to_log(self.log_opt["log_name"],self.brosbag_path)) if self.brosbag_path is not None else None
        # elif self.mode == "SIMULATION":
            # import socket
            # self.s = socket.socket()
            # self.s.bind(("127.0.0.1", 65432))
            # self.s.listen()
            # print(f"waiting to connect") self.connection, addr = self.s.accept()
            # print(f"connected by {addr}")


        self.detector = ConeDetector(config["cone_detector_opt"])
        self.localizer = ConeLocalizer(config["cone_localizer_opt"])
        self.path_planner = PathPlanner(config["path_planner_opt"])
        self.logger = Logger(log_name=self.log_opt["log_name"], log_folder_name=self.log_opt["log_folder_name"], main_folder_path=self.main_log_folder)
        self.logger.log("CONE_DETECTOR_CONFIGURATION", config) # log config

    def run(self):
        print("STARTING CONE DETECTION")
        self.initialize()

        if self.mode == "SIMULATION":
            self.run_simulation()
            return

        while True:
            if self.mode == "BROSBAG":
                image = self.read_log_image()
            elif self.mode == "CAMERA":
                image = self.read_zed_image()

            bbox_preds = self.detector.process_image(image).cpu().detach().numpy()
            world_preds = self.localizer.project_bboxes(bbox_preds)
            path = self.path_planner.find_path(world_preds)

            self.output_queue.put(path)

            cone_classes = bbox_preds[:,5].astype(int)
            data = {
                "bboxes": bbox_preds,
                "world_preds": world_preds,
                "cone_classes": cone_classes,
                "path": path
            }
            self.logger.log("CONE_DETECTOR_FRAME", data)

    def run_simulation(self):
        listen_address = "localhost", 50000
        with connection.Client(listen_address) as conn:
            try:
                while True:
                    item = conn.recv()
                    print("item: ", item)
            except EOFError:
                pass

        # with self.connection:
            # while True:

                # data = b''
                # while True:
                    # block = self.connection.recv(128)
                    # if not block: break
                    # data += block
                # print("data: ", data)
                # print("data_len: ", len(data))
                # sys.exit(0)
                # path = self.path_planner.find_path(world_preds)
                # self.output_queue.put(path)
                # data = {
                    # "world_preds": world_preds,
                    # "path": path
                # }
                # self.logger.log("CONE_DETECTOR_GRAME", data)

    def read_zed_image(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrive_image(self.zed_image, sl.VIEW.LEFT)
            image = self.zed_image.get_data()
            return image
        raise Exception("COULDN'T RETRIEVE IMAGE")

    def read_log_image(self):
        msg_t, (msg_type, data) = next(self.brosbag_gen)
        assert msg_type == "CONE_DETECTOR_FRAME"
        return data["image"]

    def read_cones_from_network(self):
        pass



