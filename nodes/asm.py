from enum import Enum

class AS(Enum):
	OFF = 0
	READY = 1
	DRIVING = 2
	FINISHED = 3
	EMERGENCY = 4

class ASM():
	"""
	OFF -> DSH_STATUS -> READY
	READY -> RES_GO -> 
	"""
	def __init__(self):
		self.AS = AS.READY

	def update(self, go_signal):
		if self.AS == AS.OFF:
			pass
		elif self.AS == AS.READY:
			if go_signal == 1:
				self.AS = AS.DRIVING

		elif self.AS == AS.DRIVING:
			pass
		elif self.AS == AS.FINISHED:
			pass
		elif self.AS == AS.EMERGENCY:
			pass