import cv2
from signal import signal
from signal import SIGTERM
import numpy as np
from config import can_config, path_planner_opt, VisionNodeMsgPorts
from nodes.vision_node import VisionNode
from algorithms import path_planning


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
    # SOLVE FOLDER BS, add DEBUG MODE to vision node
    path_planner = path_planning.PathPlanner(path_planner_opt)
    vision_node = VisionNode(main_log_folder=main_log_folder, mode="DEBUG")
    vision_node.start()
    print("VISION NODE STARTED")
