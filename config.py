from pathlib import Path
import numpy as np
import os
from enum import IntEnum, auto

##################
# GENERAL CONFIG #
##################

file_path = Path(__file__).parent

config = {
    "main_log_folder": Path.cwd() / Path("data/bros_logs")
}

###########################
# INTERNODE COMMUNICATION #
###########################


class NodePorts(IntEnum):
    CAN1 = 10000
    CAN2 = 20000
    VISION = 30000
    MISSION = 40000


class CAN1NodeMsgPorts(IntEnum):
    WHEEL_SPEED = NodePorts.CAN1
    STEERING_ANGLE = auto()
    MISSION = auto()
    START_BUTTON = auto()
    CAR_STATUS = auto()
    TSON_BUTTON = auto()
    ASMS_OUT = auto()


class CAN2NodeMsgPorts(IntEnum):
    GO_SIGNAL = NodePorts.CAN2
    EMERGENCY_SIGNAL = auto()
    SWITCH_SIGNAL = auto()
    POSITION = auto()
    VELOCITY = auto()
    EULER = auto()
    INS_STATUS = auto()
    ACCELERATION = auto()
    EULER_ACCURACY = auto()
    POSITION_ACCURACY = auto()
    VELOCITY_ACCURACY = auto()
    ACCELERATOR_POS = auto()


class VisionNodeMsgPorts(IntEnum):
    CONE_PREDS = NodePorts.VISION


class MissionNodeMsgPorts(IntEnum):
    WHEEL_SPEED_CMD = NodePorts.MISSION
    STEERING_ANGLE_CMD = auto()
    KSICHT_STATUS = auto()


class MissionValue(IntEnum):
    NoValue = 0,
    Acceleration = 1,
    Skidpad = 2,
    Autocross = 3,
    Trackdrive = 4,
    EBS_Test = 5,
    Inspection = 6,
    Manual = 7,
    Disco = 8,
    Donuts = 9


class AS(IntEnum):
    OFF = 0
    READY = 1
    DRIVING = 2
    FINISHED = 3
    EMERGENCY = 4


class CarStatus(IntEnum):
    NOT_READY = 0,
    TS_READY = 1,
    PRECHARGE = 2,
    TS_ON = 3,
    WAITING_FOR_RTDS = 4,
    STARTED = 5


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

##############
# CAN 1 NODE #
##############

can1_config = {
    "log_folder_name": "AS",
    "log_name": "can1",
    "log_messages": True
}

##############
# CAN 2 NODE #
##############

can2_config = {
    "log_folder_name": "AS",
    "log_name": "can2",
    "log_messages": True
}

#######################
# MISSION NODE CONFIG #
#######################

mission_opt = {
    "frequency": 30,
    "log_name": "mission",
    "log_folder_name": "AS"
}

##########################
# CAN SENDER NODE CONFIG #
##########################

can_sender_config = {
    "log_folder_name": "AS",
    "log_name": "cansender",
    "log_messages": True,
    "frequency": 200
}

######################
# VISION NODE CONFIG #
######################

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
    "n_steps": 10
}

vision_node_config = {
    "cone_detector_opt": cone_detector_opt,
    "cone_localizer_opt": cone_localizer_opt,
    "path_planner_opt": path_planner_opt,
    "log_name": "vision",
    "log_folder_name": "AS",
    "simulate_images": False
}

###########################
# PERF LOGGER NODE CONFIG #
###########################

perf_logger_config = {
    "log_name": "computer_status",
    "log_folder_name": "AS",
    "gpu_processes": [
        "SLAM_NODE",
        "VISION_NODE",
        "LIDAR_NODE"
    ]
}

###########################
# INSPECTION MISSION CONFIG #
###########################

inspection_config = {
    "max_steering_angle": 90,
    "speed_set_point": 500.0,
    "duration": 25.,
    "n_periods": 4.
}
