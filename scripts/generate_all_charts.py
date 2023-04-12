import argparse
from pathlib import Path

from tvojemama.log_to_chart import log_to_chart

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bros_data', type=str, default="data/bros_logs")
    args = parser.parse_args()

    bros_data_path = Path(args.bros_data)

    for log_folder in bros_data_path.iterdir():
        AS_folder = Path(log_folder) / Path("AS")

        try:
            log_to_chart(AS_folder, show=False)
        except e:
            print("log error")
            print(e)
