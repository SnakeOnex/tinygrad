from enum import IntEnum
from internode_communication import create_subscriber_socket, update_subscription_data, create_publisher_socket, publish_data
from config import CAN1NodeMsgPorts, CAN2NodeMsgPorts, MissionNodeMsgPorts, MissionValue


class AS(IntEnum):
    OFF = 0
    READY = 1
    DRIVING = 2
    FINISHED = 3
    EMERGENCY = 4


class ASM():
    """
    OFF -> DSH_STATUS -> READY
    READY -> RES_GO -> DRIVING
    DRIVING -> FINISHED -> FINISHED
    DRIVING -> ERROR -> EMERGENCY
    """

    def __init__(self):

        # ASM required sockets
        self.start_button_socket = create_subscriber_socket(
            CAN1NodeMsgPorts.START_BUTTON)
        self.go_signal_socket = create_subscriber_socket(
            CAN2NodeMsgPorts.GO_SIGNAL)
        self.switch_signal_socket = create_subscriber_socket(
            CAN2NodeMsgPorts.SWITCH_SIGNAL)
        self.ins_status_socket = create_subscriber_socket(
            CAN2NodeMsgPorts.INS_STATUS)
        self.ksicht_status_socket = create_publisher_socket(
            MissionNodeMsgPorts.KSICHT_STATUS)
        self.emergency_signal_socket = create_subscriber_socket(
            CAN2NodeMsgPorts.EMERGENCY_SIGNAL)
        self.mission_socket = create_subscriber_socket(
            CAN1NodeMsgPorts.MISSION)

        # ASM required data
        self.start_button = 0
        self.go_signal = 0
        self.ins_status = (0., 0., 0.)
        self.switch = 0
        self.last_ksicht_status = [0, 0, 0, 0, 0, 0, 0, 0]
        self.emergency_signal = 0
        self.mission_num = MissionValue.NoValue

        # AS state
        self.AS = AS.OFF

    def update_asm_status(self):
        self.start_button = update_subscription_data(
            self.start_button_socket, self.start_button)
        self.go_signal = update_subscription_data(
            self.go_signal_socket, self.go_signal)
        self.ins_status = update_subscription_data(
            self.ins_status_socket, self.ins_status)
        self.switch = update_subscription_data(
            self.switch_signal_socket, self.switch)
        self.emergency_signal = update_subscription_data(
            self.emergency_signal_socket, self.emergency_signal)

        if self.emergency_signal == 1:
            self.set_emergency()

        if self.AS == AS.OFF:
            if self.start_button == 1:
                self.AS = AS.READY
                print("ASM -> AS.READY")

        elif self.AS == AS.READY:
            if self.go_signal == 1:
                self.AS = AS.DRIVING
                print("ASM -> AS.DRIVING")

    def send_ksicht_status(self):
        publish_data(self.ksicht_status_socket, [
            self.AS.value, self.mission_num, 0, self.switch, self.ins_status[0], self.ins_status[1], 0, 0])

    def get_mission_num(self):
        self.mission_num = update_subscription_data(
            self.mission_socket, self.mission_num)
        return self.mission_num

    def set_finished(self):
        self.AS = AS.FINISHED
        print("ASM -> AS.FINISHED")

    def set_emergency(self):
        self.AS = AS.EMERGENCY
        print("ASM -> AS.EMERGENCY")

    def get_state(self):
        return self.AS

    def is_driving(self):
        return True if self.AS == AS.DRIVING else False

    # def start_button(self):
    #     if self.AS == AS.OFF:
    #         self.AS = AS.READY

    # def go_signal(self):
    #     if self.AS == AS.READY:
    #         self.AS = AS.DRIVING

    # def finished(self):
    #     if self.AS == AS.DRIVING:
    #         self.AS = AS.FINISHED

    # def emergency(self):
    #     if self.AS == AS.DRIVING:
    #         self.AS = AS.EMERGENCY
