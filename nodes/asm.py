from enum import IntEnum

from config import AS, CarStatus


class ASM():
    """
    OFF -> DSH_STATUS -> READY
    READY -> RES_GO -> DRIVING
    DRIVING -> FINISHED -> FINISHED
    DRIVING -> ERROR -> EMERGENCY
    """

    def __init__(self):
        # self.AS = AS.READY
        self.AS = AS.OFF

    def update(self, tson_button, asms_out, go_signal, car_status, finished):
        """
        args:
          tson_button: 1 if ts_on_button is pressed
          asms_out: 1 if AS key is set to ON
          go_signal: 1 if go_signal is set to ON
          car_status: indicates EBSS car status (via CarStatus enum)
          finished: True if the car has finished the track
        """
        if asms_out == 0 and self.AS != AS.OFF:
            self.AS = AS.OFF
            print("ASM -> AS.OFF")

        if self.AS == AS.OFF:
            if tson_button == 1:
                self.AS = AS.READY
                print("ASM -> AS.READY")

        elif self.AS == AS.READY:
            if go_signal == 1:
                self.AS = AS.DRIVING
                print("ASM -> AS.DRIVING")

            if car_status == CarStatus.NOT_READY:
                self.AS = AS.EMERGENCY
                print("ASM -> AS.EMERGENCY")

        elif self.AS == AS.DRIVING:
            if car_status == CarStatus.NOT_READY:
                self.AS = AS.EMERGENCY
                print("ASM -> AS.EMERGENCY")

            if finished == True:
                self.AS = self.AS.FINISHED
                print("ASM -> AS.FINISHED")

        elif self.AS == AS.FINISHED:
            pass
        elif self.AS == AS.EMERGENCY:
            pass

    def raise_emergency(self):
        self.AS = AS.EMERGENCY
        print("ASM -> AS.EMERGENCY")
