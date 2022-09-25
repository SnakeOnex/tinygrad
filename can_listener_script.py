from pathlib import Path
import can
from can_lib.can_interface import CanInterface
from can_lib.can_ids import CanIds

if __name__ == '__main__':
    # 1. INITIALIZATION
    CAN1 = CanInterface(Path("can_lib/D1.json"), 0)
    ## CAN_DB
    # candb = CanDB([Path("D1.json")])

    ## CAN_BUS
    bus = can.Bus(interface='socketcan',
                  channel='CAN1_powertrain',
                  receive_own_messages=True)

    ## send CAN message
    values = [1,1, 0, 10, 11, 12, 0]
    CAN1.send_can_msg(values, CanIds.MCR_ActualValues_A)

    # iterate over received messages
    for msg in bus:
        print(f"{msg.arbitration_id:X}: {msg.data}")
        values = read_can_msg(msg)
        print(f"values: {values}")

    # or use an asynchronous notifier
    # notifier = can.Notifier(bus, [can.Logger("recorded.log"), can.Printer()])
