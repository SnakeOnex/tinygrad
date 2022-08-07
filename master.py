import numpy as np
import subprocess
import pickle
import sys

if __name__ == '__main__':
    # p = subprocess.run(
            # "./dummy_process.py",
            # # input=3,
            # stdin=subprocess.PIPE,
            # stdout=subprocess.PIPE,
            # stderr=subprocess.PIPE
    # )
    p = subprocess.Popen(
            "./dummy_process.py",
            # input=3,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
    )

    std_out, std_err = p.communicate(input=b'ksicht')

    try:
        err = p.stderr.read()
    except ValueError:
        pass

    sys.exit(0)

    if len(p.stderr) > 0:
        print(f"STDERR: \n{p.stderr.decode('ascii')}")

    if len(p.stdout):
        print(f"STDOUT: \n{p.stdout}")
        # print(f"in_err: {type(p.stdout)}", file=sys.stderr)
        print(f"in_err: {type(p.stdout)}")
        out = pickle.loads(p.stdout)

        print(out)
