from pathlib import Path
from pycandb.can_interface import CanInterface

if __name__ == '__main__':
    # 1. INITIALIZATION
    CAN1 = CanInterface(Path("can_lib/D1.json"), 0)
    # CAN_DB
    # candb = CanDB([Path("D1.json")])

    print(CAN1.canids)

    can_ids = CAN1.get_canids_dict()

    # send CAN message
    values = [1, 1, 0, 10, 11, 12, 0]
    CAN1.send_can_msg(values, can_ids["MCR_ActualValues_A"])

    # iterate over received messages
    for msg in CAN1.bus:
        print(f"{msg.arbitration_id:X}: {msg.data}")
        values = CAN1.read_can_msg(msg)
        print(f"values: {values}")

    # or use an asynchronous notifier
    # notifier = can.Notifier(bus, [can.Logger("recorded.log"), can.Printer()])
