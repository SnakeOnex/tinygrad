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

    def update(self, start_button, go_signal, emergency_signal, car_status, finished):
        if emergency_signal == 1:
            self.AS = AS.EMERGENCY
            return

        if self.AS == AS.OFF:
            if start_button == 1:
                self.AS = AS.READY
                print("ASM -> AS.READY")

        elif self.AS == AS.READY:
            if go_signal == 1:
                self.AS = AS.DRIVING
                print("ASM -> AS.DRIVING")

        elif self.AS == AS.DRIVING:
            if car_status == CarStatus.NOT_READY:
                self.AS = AS.EMERGENCY

            if finished == True:
                self.AS = self.AS.FINISHED  
                print("ASM -> AS.FINISHED")
                
        elif self.AS == AS.FINISHED:
            pass
        elif self.AS == AS.EMERGENCY:
            pass

    def start_button(self):
        if self.AS == AS.OFF:
            self.AS = AS.READY

    def go_signal(self):
        if self.AS == AS.READY:
            self.AS = AS.DRIVING

    def finished(self):
        if self.AS == AS.DRIVING:
            self.AS = AS.FINISHED

    def emergency(self):
        if self.AS == AS.DRIVING:
            self.AS = AS.EMERGENCY
