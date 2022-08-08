#!/home/snakeone/anaconda3/envs/ultra-yolov3/bin/python
import sys
import numpy as np
import pickle
import time
import signal

class DummyProcess():
    def __init__(self):
        pass

    def receive_from_pipe(self, msg):
        print(msg)

def receive_signal(signum, stack):
    # sys.stderr.buffer.write(b'pytel')
    sys.stdout.buffer.write(b'ksicht')
    return
    # sys.stderr.buffer.write(signum)
    # sys.stderr.buffer.write(stack)

def main():
    sys.stdout.buffer.write(b'error dummy\n')
    signal.signal(signal.SIGUSR1, receive_signal)

    while True:
        sys.stdout.buffer.write(b'waiting\n')
        time.sleep(2.2)
        sys.stdout.buffer.write(b'waiting\n')
        break

    sys.exit(0)

    inp = sys.stdin.buffer.read()

    time.sleep(1.2)
    sys.stdout.buffer.write(inp)


if __name__ == '__main__':

    try:
        main()
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
