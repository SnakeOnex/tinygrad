import numpy as np
import multiprocessing as mp
import multiprocessing.connection as connection
import time
import sys
import zmq
import pickle

from cones.cone_detector import ConeDetector
from cones.cone_localizer import ConeLocalizer
from tvojemama.logger import Logger, LogReader, name_to_log
from config import vision_node_config as config
from config import tcp_config as tcp
from internode_communication import create_publisher_socket, publish_data
from config import VisionNodeMsgPorts


class VisionNode(mp.Process):
    def __init__(self, main_log_folder, mode):
        mp.Process.__init__(self)
        self.main_log_folder = main_log_folder
        self.mode = mode

    def initialize(self):
        print("INITTING")
        # zed setup or brosbag setup
        if not config["simulation_mode"]:
            import pyzed.sl as sl
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps = 30
            self.zed_image = sl.Mat()
            self.runtime_parameters = sl.RuntimeParameters()
            self.detector = ConeDetector(config["cone_detector_opt"])
            self.localizer = ConeLocalizer(config["cone_localizer_opt"])

        self.logger = Logger(log_name=config["log_name"], log_folder_name=config["log_folder_name"], main_folder_path=self.main_log_folder)
        self.logger.log("VISION_CONFIGURATION", config)  # log config

        self.cone_preds_socket = create_publisher_socket(VisionNodeMsgPorts.CONE_PREDS)

    def run(self):
        print("STARTING CONE DETECTION")
        self.initialize()

        if config["simulation_mode"]:
            self.run_simulation()
            return

        while True:
            if self.mode == "BROSBAG":
                image = self.read_log_image()
            elif self.mode == "CAMERA":
                image = self.read_zed_image()

            bbox_preds = self.detector.process_image(image).cpu().detach().numpy()

            world_preds = self.localizer.project_bboxes(bbox_preds)

            cone_classes = bbox_preds[:, 5].astype(int)
            data = {
                "bboxes": bbox_preds,
                "world_preds": world_preds,
                "cone_classes": cone_classes,
            }
            self.logger.log("CONE_DETECTOR_FRAME", data)

    def run_simulation(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(tcp["TCP_HOST"] + ":" + tcp["VISION_PORT"])
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        image = None

        if config["simulate_images"]:
            image = (np.random.rand(720,1280,3) * 255).astype(np.uint8)

        while True:
            data = socket.recv()
            world_preds = pickle.loads(data)
            publish_data(self.cone_preds_socket, world_preds)
            self.logger.log("CONE_DETECTOR_FRAME", {"image": image, "world_preds": world_preds})

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
