import multiprocessing
import time
from .cone_detector_config import config
from cones.cone_detector import ConeDetector
from cones.cone_localizer import ConeLocalizer
from tvojemama.logger import Logger, LogReader

class ConeDetectionNode(multiprocessing.Process):
    def __init__(self, output_queue, main_log_folder, brosbag_path=None):
        self.output_queue = output_queue

        # zed setup or brosbag setup
        if brosbag_path is None:
            import pyzed.sl as sl
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps = 60
            self.zed_image = sl.Mat()
            self.runtime_parameters = sl.RuntimeParameters()
        else:
            self.brosbag_gen = LogReader(brosbag_path) if brosbag_path is not None else None

        self.detector = ConeDetector(config["cone_detector_opt"])
        self.localizer = ConeLocalizer(config["cone_localizer_opt"])
        log_opt = config["logging_opt"]
        self.logger = Logger(log_name=log_opt["log_name"], log_folder_name=log_opt["log_folder_name"], main_folder_path=main_log_folder)
        self.logger.log("CONE_DETECTOR_CONFIGURATION", config) # log config

    def run(self):
        while True:
            if self.brosbag_gen is None:
                image = read_zed_image()
            else:
                image = read_log_image()

            bbox_preds = self.detector(image)
            world_preds = self.localizer.project_bboxes(self.bbox_preds)
            self.output_queue.put(world_preds)

    def read_zed_image(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrive_image(self.zed_image, sl.VIEW.LEFT)
            image = self.zed_image.get_data()
            return image
        raise Exception("COULDN'T RETRIEVE IMAGE")

    def read_log_image(self):
        msg_t, (msg_type, data) = next(self.brosbag_gen)
        assert msg_type == "CONE_DETECTOR_CONFIGURATION"
        return data["image"]
