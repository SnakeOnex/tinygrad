import multiprocessing as mp
from pycandb.can_interface import CanInterface
from internode_communication import create_publisher_socket, publish_data
from config import CAN2NodeMsgPorts


class Can2Node(mp.Process):
    def __init__(self):
        mp.Process.__init__(self)
        self.bus_name = "can2"

    def initialize(self):
        self.CAN2 = CanInterface("data/D1.json", 1, False)

        self.go_signal_socket = create_publisher_socket(CAN2NodeMsgPorts.GO_SIGNAL)
        self.position_socket = create_publisher_socket(CAN2NodeMsgPorts.POSITION)
        self.euler_socket = create_publisher_socket(CAN2NodeMsgPorts.EULER)

        self.message_callbacks = {
            self.CAN2.name2id["RES_Status"]: self.receive_RES_Status,
            self.CAN2.name2id["INS_D_EKF_EULER"]: self.receive_INS_D_EKF_EULER,
            self.CAN2.name2id["INS_D_EKF_POS"]: self.receive_INS_D_EKF_POS,
        }

    def run(self):
        self.initialize()

        while True:
            msg = self.CAN2.recv_can_msg()

            if msg.arbitration_id in self.message_callbacks:
                values = self.CAN2.read_can_msg(msg)
                # print(f"received msg: {self.CAN2.id2name[msg.arbitration_id]} values: {values}")
                self.message_callbacks[msg.arbitration_id](values)

    # CAN MESSAGE RECEIVE CALLBACK FUNCTIONS
    def receive_RES_Status(self, values):
        # print(f"received RES status, values: ", values)
        go_signal = values[2]
        # print("go_signal: ", go_signal)
        publish_data(self.go_signal_socket, go_signal)

    def receive_INS_D_EKF_POS(self, values):
        latitude = values[0]
        longitude = values[1]
        publish_data(self.position_socket, (latitude, longitude))

    def receive_INS_D_EKF_EULER(self, values):
        pitch, yaw, roll = values
        publish_data(self.euler_socket, (pitch, yaw, roll))
