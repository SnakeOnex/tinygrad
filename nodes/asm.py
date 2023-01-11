from enum import IntEnum

class AS(IntEnum):
	OFF = 0
	READY = 1
	DRIVING = 2
	FINISHED = 3
	EMERGENCY = 4

class ASM():
	"""
	OFF -> DSH_STATUS -> READY
	READY -> RES_GO -> DRIVING
	DRIVING -> FINISHED -> FINISHED
	DRIVING -> ERROR -> EMERGENCY
	"""
	def __init__(self):
		# self.AS = AS.READY
		self.AS = AS.OFF

	def update(self, start_button, go_signal, finished):
		if self.AS == AS.OFF:

			if start_button == 1:
				self.AS = AS.READY
				print("ASM -> AS.READY")

		elif self.AS == AS.READY:
			if go_signal == 1:
				self.AS = AS.DRIVING
				print("ASM -> AS.DRIVING")

		elif self.AS == AS.DRIVING:
			if finished == True:
				self.AS = self.AS.FINISHED	
				print("ASM -> AS.FINISHED")
				
		elif self.AS == AS.FINISHED:
			pass
		elif self.AS == AS.EMERGENCY:
			pass

	def start_button(self):
		if self.AS == AS.OFF:
			self.AS = AS.READY

	def go_signal(self):
		if self.AS == AS.READY:
			self.AS = AS.DRIVING

	def finished(self):
		if self.AS == AS.DRIVING:
			self.AS = AS.FINISHED

	def emergency(self):
		if self.AS == AS.DRIVING:
			self.AS = AS.EMERGENCY
