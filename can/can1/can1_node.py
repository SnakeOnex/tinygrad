import multiprocessing as mp

class Can1Node(mp.Process):
    def __init__(self):
        mp.Process.__init__(self)
        self.mode = "SIMULATION" # "SIMULATION", "CAN"

    def run(self):
        pass


