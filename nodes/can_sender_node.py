import multiprocessing as mp
import subprocess
import time

from tvojemama.logger import Logger
from internode_communication import create_subscriber_socket, update_subscription_data, create_publisher_socket, publish_data
from pycandb.can_interface import CanInterface

from config import can_config, tcp_config
from config import MissionNodeMsgPorts
from config import can_sender_config as opt


class CanSenderNode(mp.Process):
    def __init__(self, main_log_folder):
        mp.Process.__init__(self)
        self.frequency = opt["frequency"]
        self.main_log_folder = main_log_folder
        self.CAN1 = CanInterface(can_config["CAN_JSON"], can_config["CAN1_ID"], False)
        self.CAN2 = CanInterface(can_config["CAN_JSON"], can_config["CAN2_ID"], False)

        self.loop_counter = 0

        self.wheel_speed_cmd = 0.
        self.steering_angle_cmd = 0.
        self.ksicht_status = (0,0,0,0,0,0,0,0)

        # CAN msgs values
        self.ksicht_status_values = [self.ksicht_status[0], self.ksicht_status[1], 0, 0, 0, 0, 0, 0]
        self.motor_setpoints_values = [0, 0, 0, 0, self.wheel_speed_cmd, 0]
        self.steering_control_values = [self.steering_angle_cmd]

    def initialize(self):
        self.logger = Logger(
            log_name=opt["log_name"],
            log_folder_name=opt["log_folder_name"],
            main_folder_path=self.main_log_folder
        )

        # Mission node message subscriptions
        self.wheel_speed_cmd_socket = create_subscriber_socket(MissionNodeMsgPorts.WHEEL_SPEED_CMD)
        self.steering_angle_cmd_socket = create_subscriber_socket(MissionNodeMsgPorts.STEERING_ANGLE_CMD)
        self.ksicht_status_socket = create_subscriber_socket(MissionNodeMsgPorts.KSICHT_STATUS)
        self.init_res()

    def update_data(self):
        self.wheel_speed_cmd = update_subscription_data(self.wheel_speed_cmd_socket, self.wheel_speed_cmd)
        self.steering_angle_cmd = update_subscription_data(self.steering_angle_cmd_socket, self.steering_angle_cmd)
        self.ksicht_status = update_subscription_data(self.ksicht_status_socket, self.ksicht_status)

        # # update values list for the CAN messages
        self.ksicht_status_values = self.ksicht_status

        self.motor_setpoints_values[4] = self.wheel_speed_cmd
        self.steering_control_values[0] = self.steering_angle_cmd

    def run(self):
        self.initialize()
        while True:
            start_time = time.perf_counter()

            # update data
            self.update_data()

            # 10 hz messages
            if self.loop_counter % 20 == 0:
                self.CAN1.send_can_msg(self.ksicht_status_values.copy(), self.CAN1.name2id["XVR_Status"])
                self.logger.log("XVR_Status", self.ksicht_status_values)

                # Lenze inverters require extended CAN address ID
                self.CAN1.send_can_msg(self.motor_setpoints_values.copy(), self.CAN1.name2id["XVR_SetpointsMotor_A"], is_extended_id=True)
                self.logger.log("XVR_SetpointsMotor_A", self.motor_setpoints_values)

            # 200 hz messages
            self.CAN1.send_can_msg(self.steering_control_values.copy(), self.CAN1.name2id["XVR_Control"])
            self.logger.log("XVR_Control", self.steering_control_values)

            self.loop_counter += 1

            end_time = time.perf_counter()
            time_to_sleep = (1. / self.frequency) - (end_time - start_time)
            if time_to_sleep > 0.:
                time.sleep(time_to_sleep)

    def init_res(self):
        data = [0x01, 0]
        self.CAN2.send_can_msg(data.copy(), self.CAN2.name2id["XVR_NMT_Mode_Control"])

