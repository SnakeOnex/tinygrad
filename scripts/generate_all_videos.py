import argparse
from pathlib import Path

from tvojemama.log_to_video import log_to_video

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bros_data', type=str, default="data/bros_logs")
    args = parser.parse_args()

    bros_data_path = Path(args.bros_data)

    for log_folder in bros_data_path.iterdir():
        AS_folder = Path(log_folder) / Path("AS")

        if ".mp4" not in [file.suffix for file in AS_folder.iterdir()]:
            log_to_video(AS_folder)
            print(f"processing {log_folder}")
        else:
            print(f"skipping {log_folder}")
