import numpy as np
import pickle

import sys
import time
import os
import subprocess
import signal


if __name__ == '__main__':

    p1_stdout = subprocess.PIPE

    p1 = subprocess.Popen(
            "./dummy_process.py",
            # input=3,
            shell=True,
            stdin=subprocess.PIPE,
            # stdout=p1_stdout,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
    )

    p2 = subprocess.Popen(
            "./dummy_process.py",
            # input=3,
            stdin=p1_stdout,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
    )
    time.sleep(0.2)


    print("p1: ", p1.pid)
    print("p2: ", p2.pid)

    p1_in = b'pliv'
    print("p1 in: ", p1_in)
    p1.stdin.write(p1_in)
    # std_out, std_err = p1.communicate(input=p1_in)

    time.sleep(0.1)

    for line in iter(p1.stdout.readline, b''):
        print("line: ", line)

    time.sleep(2)
    for line in iter(p1.stdout.readline, b''):
        print("line2: ", line)

    sys.exit(0)

    try:
        err = p1.stdout.readline()
        print("p1 out: ", err)
    except ValueError:
        print("no err input")

    p1.send_signal(signal.SIGUSR1)
    print("sent signal")
    time.sleep(0.1)

    sys.exit(0)

    try:
        out = p1.stdout.read()
        print("p1 out: ", out)
    except ValueError:
        print("no out input")

    sys.exit(0)

    if len(p.stderr) > 0:
        print(f"STDERR: \n{p.stderr.decode('ascii')}")

    if len(p.stdout):
        print(f"STDOUT: \n{p.stdout}")
        # print(f"in_err: {type(p.stdout)}", file=sys.stderr)
        print(f"in_err: {type(p.stdout)}")
        out = pickle.loads(p.stdout)

        print(out)


