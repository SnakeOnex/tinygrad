import cv2
import numpy as np
import pyzed.sl as sl
from algorithms import path_planning
from cones.cone_detector import ConeDetector
from cones.cone_localizer import ConeLocalizer
from config import vision_node_config as config
from config import tcp_config as tcp


CONE_COLORS = {0.: (0, 255, 255),
               1.: (255, 0, 0),
               2.: (255, 165, 0),
               3.: (255, 165, 0), }


def draw_bbox_to_im(image, conepreds):
    for pred in conepreds:
        topleft = pred[:2]
        bottomright = pred[2:4]
        image = cv2.rectangle(image, topleft, bottomright, CONE_COLORS[pred[4]], thickness=3)
    return image


def main(args):
    cv2.namedWindow("Perception")
    show_bboxes = False
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 60
    zed_image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    detector = ConeDetector(config["cone_detector_opt"])
    zed.open()

    while True:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(zed_image, sl.VIEW.LEFT)
            image = zed_image.get_data()
        bbox_preds = detector.process_image(image)
        if bbox_preds is not None:
            bbox_preds = bbox_preds.cpu().detach().numpy()
            if show_bboxes:
                draw_bbox_to_im(image)

        cv2.imshow('Perception', image)
        key = cv2.waitKey(5)
        if key == ord("k"):
            break
        elif key == ord("b"):
            show_bboxes = not show_bboxes

    cv2.destroyAllWindows()

    zed.close()
