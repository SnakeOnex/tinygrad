import numpy as np
import time

from .math_helpers import angle_to_vector, global_to_local


class TrackMarshall():
	"""
	Keeps track the world state in terms of the ruleset, eg.: number of cones hit, start time, finish time, etc	
	"""

	def __init__(self, state):
		self.state = state

		# cone mask of hit cones
		self.cones_mask = np.zeros((self.state.cones_world.shape[0]))

		self.is_on_start_line = False
		self.is_on_finish_line = False

		self.is_starting = False # is on start line
		self.has_started = False # has left start line

		self.is_finishing = False
		self.has_finished = False

		self.start_time = None
		self.race_time = 0.0

	def update(self):
		# 1. compute cone hits
		## transform cones into local coordinates system of the car (0,0 is in middle of front axle)
		cones_local = global_to_local(np.array(self.state.cones_world[:, 0:2]), self.state.car_pos, self.state.heading)

		## shift everything in the reverse heading direction by 0.5 meters (to center of the car)
		heading_vec = angle_to_vector(self.state.heading)
		# cones_local -= (self.state.wheel_base/2) * heading_vec

		## compute manhattan norm of all the cones
		cones_y_dists = abs(cones_local[:,0])
		cones_x_dists = abs(cones_local[:,1])

		collision_idxs = np.where((cones_x_dists < (self.state.car_length / 2)) & (cones_y_dists < (self.state.car_width / 2)))[0]
		self.cones_mask[collision_idxs] = 1

		# 2. compute start and finish state
		start_line_local = global_to_local(np.array(self.state.start_line), self.state.car_pos, self.state.heading)
		if (abs(start_line_local[:,0]) < 2.).all() and (abs(start_line_local[:,1]) < self.state.car_length / 2).all():
			self.is_on_start_line = True
		else:
			self.is_on_start_line = False

		finish_line_local = global_to_local(np.array(self.state.finish_line), self.state.car_pos, self.state.heading)

		if (abs(finish_line_local[:,0]) < 2.).all() and (abs(finish_line_local[:,1]) < self.state.car_length / 2).all():
			self.is_on_finish_line = True
		else:
			self.is_on_finish_line = False

		# detect start
		if not self.has_started and self.is_on_start_line:
			self.is_starting = True
			self.start_time = time.perf_counter()
		elif not self.has_started and self.is_starting and not self.is_on_start_line:
			self.has_started = True
			self.is_starting = False

		if self.has_started and self.is_on_finish_line and time.perf_counter() - self.start_time > 4:
			self.is_finishing = True
		elif self.has_started and self.is_finishing and not self.is_on_finish_line and not self.has_finished:
			self.has_finished = True
			self.finish_time = time.perf_counter()

		if (self.has_started or self.is_starting) and not self.has_finished:
			self.race_time = time.perf_counter() - self.start_time
		elif self.has_started and self.has_finished:
			self.race_time = self.finish_time - self.start_time

		if not self.has_started and self.is_on_start_line:
			self.has_started = True


		# self.debug = (self.is_on_start_line, self.is_on_finish_line, self.has_started, self.has_finished, self.race_time)
		self.debug = {
			"race_time"	: self.race_time,
			"mission" : self.state.mission.name,
			"tson" : self.state.tson,
			"go_signal" : self.state.go_signal,
			"cones_hit" : int(np.sum(self.cones_mask))
		}

	def ksicht(self):
		pass