import argparse
from pathlib import Path
import datetime
import os
import shutil
import sys

from tvojemama.log_to_chart import log_to_chart
from tvojemama.log_to_video import log_to_video

def latest_log_dir(root_log_dir):
    logs = os.listdir(root_log_dir)
    log_path = [datetime.datetime.strptime(log, "%d_%m_%Y-%H_%M_%S") for log in logs]
    latest_log = logs[log_path.index(max(log_path, key=lambda x: x.timestamp()))]
    last_log_dir = os.path.join(root_log_dir, latest_log)
    return last_log_dir


def clear_all_logs(root_log_dir):
    all_logs = os.listdir(root_log_dir)
    for log in all_logs:
        shutil.rmtree(os.path.join(root_log_dir, log), ignore_errors=True)


def process_log_folder(log_folder, show=False, video=False):
    AS_folder = Path(log_folder) / Path("AS")
    if ".jpg" not in [file.suffix for file in AS_folder.iterdir()]:
        print(f"generating charts for {log_folder}")
        log_to_chart(AS_folder, show=show)
    else:
        print(f"skipping charts for {log_folder}")

    if video and ".mp4" not in [file.suffix for file in AS_folder.iterdir()]:
        print(f"generating charts for {log_folder}")
        log_to_video(AS_folder)
    else:
        print(f"skipping video in {log_folder}")

def main(args):
    bros_data_path = Path(args.bros_data)
    if args.clear_all_logs:
        clear_all_logs(bros_data_path)
    else:
        if args.from_dir:
            process_log_folder(bros_data_path / Path(args.from_dir), show=args.show, video=args.video)
        elif args.latest:
            last_log_dir = latest_log_dir(bros_data_path)
            process_log_folder(last_log_dir, show=args.show, video=args.video)
        else:
            for log_folder in bros_data_path.iterdir():
                process_log_folder(log_folder, show=args.show, video=args.video)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_help = True
    parser.add_argument('--bros_data', type=str, default="data/bros_logs", help="path to bros data folder")
    parser.add_argument('--from_dir', type=str, default=None, help="path to specific log folder")
    parser.add_argument('--latest', action='store_true', default=False, help="use the latest log found by date")
    parser.add_argument('--show', action='store_true', default=False, help="show all of the figures")
    parser.add_argument('--clear_all_logs', action='store_true', default=False, help="clear all logs from the log folder")
    parser.add_argument('--video', action='store_true', default=False, help="generate video for the given logs")

    args = parser.parse_args()
    main(args)
