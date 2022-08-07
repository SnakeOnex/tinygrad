#!/home/snakeone/anaconda3/envs/ultra-yolov3/bin/python
import sys
import numpy as np
import pickle

class DummyProcess():
    def __init__(self):
        pass

    def receive_from_pipe(self, msg):
        print(msg)

if __name__ == '__main__':

    output = {
        "ksicht": np.array([1., 1.5, 2.])
    }

    # output = [1,2,3]
    a = input()
    print("a: ", a)

    out_ser = pickle.dumps(output)

    print(f"out_type: {type(out_ser)}", file=sys.stderr)
    print(f"out: {out_ser}", file=sys.stderr)

    in_ser = pickle.loads(out_ser)

    sys.stdout.buffer.write(out_ser)
    # print(out_ser, end='')
    # print(in_ser)

    # 5/0
    # print(in_ser, end='')
