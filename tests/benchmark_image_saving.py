import numpy as np
import pickle
from PIL import Image
from pathlib import Path
import matplotlib.pyplot as plt
import time
import cv2

from tvojemama.logger import Logger

def generate_random_image(img_size):
	image =(np.random.rand(*img_size) * 255).astype(np.uint8)
	return image

def benchmark_pickle(number_of_images, img_size):
	logger = Logger(log_name="benchmark", log_folder_name="AS", main_folder_path=Path(__file__).absolute().parent)

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

	start_time = time.perf_counter()
	for i in range(number_of_images):
		np.save("image_benchmark/" + f"NUMPY_{i}.npy", image)
	end_time = time.perf_counter()

	print(f"NUMPY BENCHMARK FINISHED: ")
	print(f"\tsaved {number_of_images} images of res {img_size} in {end_time - start_time:.2f} seconds.")
	print(f"\twhich is a frequency of {number_of_images/(end_time-start_time):.2f} images per second")


if __name__ == '__main__':
	test_folder_path = Path("image_benchmark").mkdir(exist_ok=True)
	# print("cwd: ", Path(__file__).absolute().parent)

	N = 500
	res = (720,1280,3)
	print(f"TESTING FOR N={N} and RES={res}")

	benchmark_pickle(N, res)
	benchmark_PIL(N, res)
	benchmark_CV2(N, res)
	benchmark_NUMPY(N, res)
