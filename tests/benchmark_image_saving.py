import numpy as np
import shutil
import pickle
from PIL import Image
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import time
import cv2

from tvojemama.logger import Logger


def generate_random_image(img_size):
    image = (np.random.rand(*img_size) * 255).astype(np.uint8)
    return image


def benchmark_pickle(number_of_images, img_size, folder_path):
    logger = Logger(log_name="benchmark", log_folder_name="AS", main_folder_path=folder_path.parent)
    image = generate_random_image(img_size)
    start_time = time.perf_counter()

    for i in range(number_of_images):
        logger.log("IMAGE", image)
    end_time = time.perf_counter()

    print(f"PICKLE BENCHMARK FINISHED: ")
    print(f"\tsaved {number_of_images} images of res {img_size} in {end_time - start_time:.2f} seconds.")
    print(f"\twhich is a frequency of {number_of_images/(end_time-start_time):.2f} images per second")


def benchmark_PIL(number_of_images, img_size):
    image = generate_random_image(img_size)

    start_time = time.perf_counter()
    for i in range(number_of_images):
        image_PIL = Image.fromarray(image)
        image_PIL.save("image_benchmark/" + f"PIL_{i}.jpg")
    end_time = time.perf_counter()

    print(f"PIL BENCHMARK FINISHED: ")
    print(f"\tsaved {number_of_images} images of res {img_size} in {end_time - start_time:.2f} seconds.")
    print(f"\twhich is a frequency of {number_of_images/(end_time-start_time):.2f} images per second")


def benchmark_CV2(number_of_images, img_size):
    image = generate_random_image(img_size)

    start_time = time.perf_counter()
    for i in range(number_of_images):
        cv2.imwrite("image_benchmark/" + f"CV2_{i}.jpg", image)
    end_time = time.perf_counter()

    print(f"CV2 BENCHMARK FINISHED: ")
    print(f"\tsaved {number_of_images} images of res {img_size} in {end_time - start_time:.2f} seconds.")
    print(f"\twhich is a frequency of {number_of_images/(end_time-start_time):.2f} images per second")


def benchmark_NUMPY(number_of_images, img_size):
    image = generate_random_image(img_size)

    loop_start_time = time.perf_counter()
    for i in range(number_of_images):
        np.save("image_benchmark/" + f"NUMPY_{i}.npy", image)
    loop_end_time = time.perf_counter()

    image_batch = [image]*number_of_images
    batch_start_time = time.perf_counter()
    np.save("image_benchmark/" + f"NUMPY_IMAGE_BATCH.npy", np.array(image_batch))
    batch_end_time = time.perf_counter()

    print(f"NUMPY BENCHMARK FINISHED: ")
    print(f"\tLOOP:")
    print(f"\tsaved {number_of_images} images of res {img_size} in {loop_end_time - loop_start_time:.2f} seconds.")
    print(f"\twhich is a frequency of {number_of_images/(loop_end_time-loop_start_time):.2f} images per second")
    print(f"\tBATCH:")
    print(f"\tsaved {number_of_images} images of res {img_size} in {batch_end_time - batch_start_time:.2f} seconds.")
    print(f"\twhich is a frequency of {number_of_images/(batch_end_time-batch_start_time):.2f} images per second")


if __name__ == '__main__':
    test_folder_path = Path("image_benchmark")
    print("Files saved at:", test_folder_path.absolute())
    test_folder_path.mkdir(exist_ok=True)
    # print("cwd: ", Path(__file__).absolute().parent)
    for arg in sys.argv[1:]:
        N = int(arg)
        res = (720, 1280, 3)
        print("--------------------------------")
        print(f"TESTING FOR N={N} and RES={res}")

        benchmark_pickle(N, res, test_folder_path)
        benchmark_PIL(N, res)
        benchmark_CV2(N, res)
        benchmark_NUMPY(N, res)

    print("removed saved files".upper())
    shutil.rmtree(test_folder_path.absolute())
