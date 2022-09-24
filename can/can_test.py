# import the library
import can
from pathlib import Path
from candb import CanDB
from can_ids import CanIds

class CanInterface():
    def __init__(self, can_json_path, channel_id, recv_self=True):
        """
        Args:
          can_json_path - path to a CanDB file
          channel_name -
        """
        self.channel_id = channel_id

        self.candb = CanDB([Path("D1.json")])
        self.bus = can.Bus(interface="socketcan", channel=channel_id, receive_own_messages=recv_self)

    def send_can_msg(self, values, message_id):
        """
        Args:
          values - list of ints
          message - candb_msg
        outs:
          out_msg - OutGoingMessage
        """
        message = self.candb.getMsgById(self.channel_id, message_id)
        data = bytearray(message.length)
        message.assemble(values, data)
        message = can.Message(data=data, arbitration_id=message.identifier, is_extended_id=True)
        bus.send(message)

    def read_can_msg(self, message):
        msg_id = message.arbitration_id
        values = self.candb.parseData(0, msg_id, message.data, message.timestamp)
        return values


if __name__ == '__main__':
    # 1. INITIALIZATION
    can_interface = CanInterface([Path("D1.json")])
    ## CAN_DB
    candb = CanDB([Path("D1.json")])

    ## CAN_BUS
    bus = can.Bus(interface='socketcan',
                  channel='CAN1_powertrain',
                  receive_own_messages=True)

    ## send CAN message
    values = [1,1, 0, 10, 11, 12, 0]
    send_can_msg(values, CanIds.MCR_ActualValues_A)

    # iterate over received messages
    for msg in bus:
        print(f"{msg.arbitration_id:X}: {msg.data}")
        values = read_can_msg(msg)
        print(f"values: {values}")

    # or use an asynchronous notifier
    # notifier = can.Notifier(bus, [can.Logger("recorded.log"), can.Printer()])
