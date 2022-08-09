import multiprocessing

from path_planning import find_path

class PathPlanningNode(multiprocessing.Process):
    def __init__(self, input_queue, output_queue, main_log_folder):
        multiprocessing.Process.__init__(self)
        self.input_queue = input_queue
        self.output_queue = output_queue

    def run(self):
        while True:
            pass
            # 1. get world preds

            # 2. compute path
