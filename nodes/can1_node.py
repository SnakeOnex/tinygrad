import multiprocessing as mp
import math

from tvojemama.logger import Logger
from pycandb.can_interface import CanInterface

from internode_communication import create_publisher_socket, publish_data
from config import CAN1NodeMsgPorts
from config import can1_config as config


class Can1Node(mp.Process):
    def __init__(self, main_log_folder):
        mp.Process.__init__(self)
        self.bus_name = "can1"
        self.main_log_folder = main_log_folder

    def initialize(self):
        self.logger = Logger(log_name=config["log_name"], log_folder_name=config["log_folder_name"], main_folder_path=self.main_log_folder)
        self.CAN1 = CanInterface("data/D1.json", 0, False)

        self.wheel_speed_socket = create_publisher_socket(CAN1NodeMsgPorts.WHEEL_SPEED)
        self.steering_angle_socket = create_publisher_socket(CAN1NodeMsgPorts.STEERING_ANGLE)
        self.mission_socket = create_publisher_socket(CAN1NodeMsgPorts.MISSION)
        self.start_button_socket = create_publisher_socket(CAN1NodeMsgPorts.START_BUTTON)
        self.car_status_socket = create_publisher_socket(CAN1NodeMsgPorts.CAR_STATUS)

        self.message_callbacks = {
            self.CAN1.name2id["MCR_ActualValues_A"]: self.receive_MCR_ActualValues_A,
            self.CAN1.name2id["SA_SteeringAngle"]: self.receive_SA_SteeringAngle,
            self.CAN1.name2id["DSH_Status"]: self.receive_DSH_Status,
            self.CAN1.name2id["EBSS_Status"]: self.receive_EBSS_Status,
        }

    def run(self):
        self.initialize()

        while True:
            msg = self.CAN1.recv_can_msg()

            if msg.arbitration_id in self.message_callbacks:
                values = self.CAN1.read_can_msg(msg)

                # log each message
                if config["log_messages"]:
                    msg_name = self.CAN1.id2name[msg.arbitration_id]
                    self.logger.log(msg_name, values)

                self.message_callbacks[msg.arbitration_id](values)

    # CAN MESSAGE RECEIVE CALLBACK FUNCTIONS
    def receive_MCR_ActualValues_A(self, values):
        WHEEL_SPEED_TO_MS = 1 / (60 * 6.7) * 2 * 0.2 * math.pi

        wheel_speed = float(values[3]) * WHEEL_SPEED_TO_MS
        publish_data(self.wheel_speed_socket, wheel_speed)

    def receive_DSH_Status(self, values):
        mission = values[3]
        start_button = values[5]
        publish_data(self.mission_socket, mission)
        publish_data(self.start_button_socket, start_button)

    def receive_SA_SteeringAngle(self, values):
        steering_angle = values[2]
        publish_data(self.steering_angle_socket, steering_angle)

    def receive_EBSS_Status(self, values):
        car_status = values[0]

        publish_data(self.car_status_socket, car_status)
