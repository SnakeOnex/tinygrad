import multiprocessing as mp
import os

from tvojemama.logger import Logger
from pycandb.can_interface import CanInterface
from utils.internode_communication import create_publisher_socket, publish_data, destroy_socket

from config import CAN2NodeMsgPorts
from config import can2_config as config


class Can2Node(mp.Process):
    def __init__(self, curr_log_folder):
        mp.Process.__init__(self)
        self.bus_name = "can2"
        self.curr_log_folder = curr_log_folder

    def initialize(self):
        self.logger = Logger(log_name=config["log_name"], log_folder_name=config["log_folder_name"],
                             curr_log_folder=self.curr_log_folder)
        self.CAN2 = CanInterface("data/D1.json", 1, recv_self=False, log_data_dir=self.curr_log_folder, log_messages=True)

        self.go_signal_socket = create_publisher_socket(CAN2NodeMsgPorts.GO_SIGNAL)
        self.emergency_signal_socket = create_publisher_socket(CAN2NodeMsgPorts.EMERGENCY_SIGNAL)
        self.switch_signal_socket = create_publisher_socket(CAN2NodeMsgPorts.SWITCH_SIGNAL)
        self.position_socket = create_publisher_socket(CAN2NodeMsgPorts.POSITION)
        self.euler_socket = create_publisher_socket(CAN2NodeMsgPorts.EULER)
        self.velocity_socket = create_publisher_socket(CAN2NodeMsgPorts.VELOCITY)
        self.ins_status_socket = create_publisher_socket(CAN2NodeMsgPorts.INS_STATUS)
        self.accel_socket = create_publisher_socket(CAN2NodeMsgPorts.ACCELERATION)
        self.orientation_acc_socket = create_publisher_socket(CAN2NodeMsgPorts.EULER_ACCURACY)
        self.position_acc_socket = create_publisher_socket(CAN2NodeMsgPorts.POSITION_ACCURACY)
        self.velocity_acc_socket = create_publisher_socket(CAN2NodeMsgPorts.VELOCITY_ACCURACY)
        self.accelerator_pos_socket = create_publisher_socket(CAN2NodeMsgPorts.ACCELERATOR_POS)

        self.message_callbacks = {
            self.CAN2.name2id["RES_Status"]: self.receive_RES_Status,
            self.CAN2.name2id["INS_D_EKF_EULER"]: self.receive_INS_D_EKF_EULER,
            self.CAN2.name2id["INS_D_EKF_POS"]: self.receive_INS_D_EKF_POS,
            self.CAN2.name2id["INS_D_EKF_VEL_BODY"]: self.receive_INS_D_EKF_VEL_BODY,
            self.CAN2.name2id["INS_D_STATUS_03"]: self.receive_INS_D_STATUS_03,
            self.CAN2.name2id["INS_D_IMU_ACCEL"]: self.receive_INS_D_IMU_ACCEL,
            self.CAN2.name2id["INS_D_EKF_ORIENT_ACC"]: self.receive_INS_D_EKF_ORIENT_ACC,
            self.CAN2.name2id["INS_D_EKF_POS_ACC"]: self.receive_INS_D_EKF_POS_ACC,
            self.CAN2.name2id["INS_D_EKF_VEL_ACC"]: self.receive_INS_D_EKF_VEL_ACC,
            self.CAN2.name2id["PDL_RawRelative"]: self.receive_PDL_Accelerator
        }

    def run(self):
        self.initialize()

        while True:
            msg = self.CAN2.recv_can_msg()

            if msg.arbitration_id in self.message_callbacks:
                values = self.CAN2.read_can_msg(msg)

                # log each message
                if config["log_messages"]:
                    msg_name = self.CAN2.id2name[msg.arbitration_id]
                    self.logger.log(msg_name, values)

                self.message_callbacks[msg.arbitration_id](values)

    # CAN MESSAGE RECEIVE CALLBACK FUNCTIONS
    def receive_RES_Status(self, values):
        emergency_signal = values[0]
        switch_signal = values[1]

        if switch_signal == 1:
            os.system("sudo shutdown 0")

        go_signal = values[2]
        radio_quality = values[10]
        publish_data(self.go_signal_socket, go_signal)
        publish_data(self.emergency_signal_socket, emergency_signal)
        publish_data(self.switch_signal_socket, switch_signal)

    def receive_INS_D_EKF_POS(self, values):
        latitude = values[0]
        longitude = values[1]
        publish_data(self.position_socket, (latitude, longitude))

    def receive_INS_D_EKF_EULER(self, values):
        roll, yaw, pitch = values
        publish_data(self.euler_socket, (pitch, yaw, roll))

    def receive_INS_D_EKF_VEL_BODY(self, values):
        vel_x, vel_y, vel_z = values
        publish_data(self.velocity_socket, (vel_x, vel_y, vel_z))

    def receive_INS_D_STATUS_03(self, values):
        heading_valid = values[2]
        velocity_valid = values[3]
        position_valid = values[4]
        publish_data(self.ins_status_socket, (heading_valid, velocity_valid, position_valid))

    def receive_INS_D_IMU_ACCEL(self, values):
        accel_x, accel_y, accel_z = values
        publish_data(self.accel_socket, (accel_x, accel_y, accel_z))

    def receive_INS_D_EKF_ORIENT_ACC(self, values):
        roll_acc, pitch_acc, yaw_acc = values
        publish_data(self.orientation_acc_socket, (roll_acc, pitch_acc, yaw_acc))

    def receive_INS_D_EKF_POS_ACC(self, values):
        lat_acc, long_acc, alt_acc = values
        publish_data(self.position_acc_socket, (lat_acc, long_acc, alt_acc))

    def receive_INS_D_EKF_VEL_ACC(self, values):
        vel_x_acc, vel_y_acc, vel_z_acc = values
        publish_data(self.velocity_acc_socket, (vel_x_acc, vel_y_acc, vel_z_acc))

    def receive_PDL_Accelerator(self, values):
        accelerator_pos = values[2]
        publish_data(self.accelerator_pos_socket, accelerator_pos)

    # def terminate(self):
    #     print("Terminating CAN2Node")
    #     destroy_socket(self.go_signal_socket)
    #     destroy_socket(self.emergency_signal_socket)
    #     destroy_socket(self.switch_signal_socket)
    #     destroy_socket(self.position_socket)
    #     destroy_socket(self.euler_socket)
    #     destroy_socket(self.velocity_socket)
    #     destroy_socket(self.ins_status_socket)
    #     destroy_socket(self.accel_socket)
    #     destroy_socket(self.orientation_acc_socket)
    #     destroy_socket(self.position_acc_socket)
    #     destroy_socket(self.velocity_acc_socket)
    #     destroy_socket(self.accelerator_pos_socket)
    #     super().terminate()
