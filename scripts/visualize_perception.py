try:
    import pyzed.sl as sl
except:
    print("No ZED SDK")

import cv2
import numpy as np
import sys

sys.path.append("..")
sys.path.append(".")

from algorithms import path_planning
from cones.visual_helpers import *
from cones.cone_detector import ConeDetector
from cones.cone_localizer import ConeLocalizer
from cones.geometry_functions import *
from config import vision_node_config as config
from config import cone_localizer_opt
from config import path_planner_opt


def main(args):
    cv2.namedWindow("Perception")
    show_bboxes = False
    draw_path = False
    # init zed
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 60
    zed_image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    zed.open()
    # init neural net
    detector = ConeDetector(config["cone_detector_opt"])
    # init projector
    localizer = ConeLocalizer(config["cone_localizer_opt"])
    # init path predictor
    path_planner = path_planning.PathPlanner(path_planner_opt)

    while True:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(zed_image, sl.VIEW.LEFT)
            image = zed_image.get_data()

            bbox_preds = detector.process_image(image)

            if bbox_preds is not None:
                bbox_preds = bbox_preds.cpu().detach().numpy()

                if show_bboxes:
                    draw_bboxes_from_preds(image, bbox_preds)

                if draw_path:
                    world_preds = localizer.project_bboxes(bbox_preds)
                    path = path_planner.find_path(world_preds)
                    path_image = homography_reject(cone_localizer_opt["hom_mat"], path)
                    for i in range(path_image.shape[0] - 1):
                        image = cv2.line(image, tuple(path_image[i].astype(int)), tuple(path_image[i + 1].astype(int)), (255, 0, 0), 4)

            cv2.imshow('Perception', image)
            key = cv2.waitKey(5)
            if key == ord("k"):
                break
            elif key == ord("b"):
                show_bboxes = not show_bboxes
            elif key == ord("p"):
                draw_path = not draw_path

    cv2.destroyAllWindows()

    zed.close()


if __name__ == "__main__":
    main()
