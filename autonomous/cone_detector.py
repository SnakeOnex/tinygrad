import multiprocessing
import time

from config import config

class ConeDetector(multiprocessing.Process):
    def __init__(self, output_queue):
        self.output_queue = output_output_queue

        self.config = config
        self.detector = ConeDetector(config["cone_detector_opt"])
        self.localizer = ConeLocalizer(config["cone_localizer_opt"])
        self.logger = Logger(log_name="cone_detector", log_folder_name="AS", main_folder_path=data_folder_path)
        self.logger.log(("CONE_DETECTOR_CONFIGURATION", config)) # log config

    def run(self):
        while True:
            # 1. load image
            image = read_zed_image()
            bbox_preds = self.detector(image)
            world_preds = self.localizer.project_bboxes(self.bbox_preds)
            self.output_queue.put(world_preds)

    def read_zed_image(self):
        raise NotImplemented

    def read_log_images(self):
        raise NotImplemented
