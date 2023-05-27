import numpy as np
import multiprocessing as mp
import multiprocessing.connection as connection
import time
import sys
import zmq
import pickle

try:
    import pyzed.sl as sl
except:
    print("No ZED SDK")

from cones.cone_detector import ConeDetector
from cones.cone_localizer import ConeLocalizer
from tvojemama.logger import Logger, LogReader, name_to_log
from config import vision_node_config as config
from config import tcp_config as tcp
from utils.internode_communication import *
from config import VisionNodeMsgPorts, CAN2NodeMsgPorts
import cv2


class VisionNode(mp.Process):
    def __init__(self, curr_log_folder, mode="RACE"):
        mp.Process.__init__(self)
        self.curr_log_folder = curr_log_folder
        self.mode = mode
        self.go_signal = 0
        self.log_images = False
        self.camera_open = False

    def initialize(self):
        print("INITTING")
        # zed setup or brosbag setup
        if self.mode == "RACE":
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps = 60
            self.zed_image = sl.Mat()
            self.runtime_parameters = sl.RuntimeParameters()
            self.detector = ConeDetector(config["cone_detector_opt"])
            self.localizer = ConeLocalizer(config["cone_localizer_opt"])
            self.zed.open()
            self.camera_open = True

        self.logger = Logger(
            log_name=config["log_name"], log_folder_name=config["log_folder_name"], curr_log_folder=self.curr_log_folder)

        # adding start global time to config
        config["start_time"] = time.time()
        self.logger.log("VISION_CONFIGURATION", config)  # log config

        # publisher sockets
        self.cone_preds_socket = create_publisher_socket(VisionNodeMsgPorts.CONE_PREDS)

        # subscriber sockets
        self.go_signal_socket = create_subscriber_socket(CAN2NodeMsgPorts.GO_SIGNAL)

    def run(self):
        print("STARTING CONE DETECTION")
        self.initialize()

        if self.mode == "SIM":
            self.run_simulation()
            return

        while True:

            if self.log_images == False:
                self.go_signal = update_subscription_data(
                    self.go_signal_socket, self.go_signal)
                if self.go_signal == 1:
                    self.log_images = True

            image = self.read_zed_image()

            bbox_preds = self.detector.process_image(image)

            if bbox_preds is not None:
                bbox_preds = bbox_preds.cpu().detach().numpy()
                world_preds = self.localizer.project_bboxes(bbox_preds)
                cone_classes = bbox_preds[:, 5].astype(int)
            else:
                bbox_preds = None
                world_preds = None
                cone_classes = None

            data = {
                "bboxes": bbox_preds,
                "world_preds": world_preds,
                "cone_classes": cone_classes,
            }

            if self.log_images:
                data["image"] = image

            publish_data(self.cone_preds_socket, world_preds)
            self.logger.log("CONE_DETECTOR_FRAME", data)

    def run_simulation(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(tcp["TCP_HOST"] + ":" + tcp["VISION_PORT"])
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        image = None

        if config["simulate_images"]:
            image = (np.random.rand(720, 1280, 3) * 255).astype(np.uint8)

        while True:
            if self.log_images == False:
                self.go_signal = update_subscription_data(
                    self.go_signal_socket, self.go_signal)
                if self.go_signal == 1:
                    self.log_images = True
            data = socket.recv()
            world_preds = pickle.loads(data)

            data = {
                "world_preds": world_preds,
            }

            if self.log_images:
                data["image"] = image

            self.logger.log("CONE_DETECTOR_FRAME", data)
            publish_data(self.cone_preds_socket, world_preds)

    def read_zed_image(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.zed_image, sl.VIEW.LEFT)
            image = self.zed_image.get_data()

            return image

    def read_log_image(self):
        msg_t, (msg_type, data) = next(self.brosbag_gen)
        assert msg_type == "CONE_DETECTOR_FRAME"
        return data["image"]

    # def terminate(self):
    #     print("Terminating Vision Node")
    #     if self.camera_open:
    #         self.zed.close()
    #     print(self.cone_preds_socket)
    #     super().terminate()
