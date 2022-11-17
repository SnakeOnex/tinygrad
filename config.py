from pathlib import Path
import numpy as np
import os

##################
# GENERAL CONFIG #
##################

file_path = Path(__file__).parent

config = {
    "main_log_folder": Path.cwd() / Path("data/bros_logs")
}

######################
# VISION NODE CONFIG #
######################

logging_opt = {
    "log_name": "cone_detector",
    "log_folder_name": "AS",
}

cone_detector_opt = {
    "cpu_weights": file_path / Path('data/weights/yolov3_tiny_640_cpu.pt'),
    "gpu_weights": file_path / Path('data/weights/yolov3_cones_640_gpu.pt'),
    "image_size": 640,
    "conf_thresh": 0.5,
    "iou_thresh": 0.15,
    "min_wh_ratio": 0.45,
    "zoom_profile": [(0.725, 0.2625, 0.55, 0.525), (0.725, 0.7375, 0.55, 0.525)]
}

cone_localizer_opt = {
    "hom_mat": np.load(file_path / Path("data/hom_mat.npy")),
    "occlusion_profile": [-6., 6., 2.5, 15.]
}

path_planner_opt = {
    "n_steps": 5
}

vision_node_config = {
    "cone_detector_opt": cone_detector_opt,
    "cone_localizer_opt": cone_localizer_opt,
    "path_planner_opt": path_planner_opt,
    "logging_opt": logging_opt
}

tcp_config = {
    "TCP_HOST": 'tcp://127.0.0.1',
    "VISION_PORT": '50000',
    "GUI_PORT": '50001',
    "CONTROLS_PORT": '50002',
    "AS_DEBUG_PORT": '50003'
}

can_config = {
    "CAN1_ID": 0,
    "CAN2_ID": 1,
    "CAN_JSON": "data/D1.json"
}

##########################
# TRACKDRIVE NODE CONFIG #
##########################
