import math
import time

class TagRead():

	def __init__(self, id, rssi, phase, freq, position, orientation):

		self.id = id
		self.rssi = rssi
		self.phase = phase
		self.freq = freq
		self.position = position
		self.orientation = orientation
		self.timestamp = time.time()
		self.fade = 1.0

class Particle():

	def __init__(self, x, y):

		self.x = x
		self.y = y
		self.w = 1

class TagState():

	def __init__(self, id):

		self.id = id

		self.tagReadObjects = []

		self.particles = createParticles(1000, -1, 5, -1, 5)

	def createParticles(self, nParticles, x1, x2, y1, y2):

		particles = []

		for _ in xrange(0, nParticles):

			xloc = random.uniform(x1, s2)
			yloc = random.uniform(y1, y2)
			particles.append(Particle(xloc, yloc)

		return particles

	def updateParticles(self, tf, model):

		for p in self.particles:

			totalProbability = 0
			totalReads = 0

			for i in self.tagReadObjects:

				relativex = p.x - i.position[0]
				relativey = p.y - i.position[1]

				roll, pitch, yaw = tf.transformations.euler_from_quaternion([i.orientation[0], i.orientation[1], i.orientation[2], i.orientation[3]])

				relativex = relativex * cos(-yaw) - relativex * sin(-yaw)
				relativey = relativey * sin(-yaw) + relativey * cos(-yaw)

				totalReads += 1
				totalProbability += model.getProbability(relativex, relativey, i.rssi, i.frequency)

			p.w = totalProbability / totalReads

	def append(self, tagRead):

		self.tagReadObjects.append(tagRead)

	def fade(seconds):

		for i in self.tagReadObjects:
			deltaTime = time.time() - i.timestamp
			if deltaTime >= seconds:
				i.fade = 0
			else:
				i.fade = deltaTime / seconds

		newList = []
		for i in self.tagReadObjects:
			if i.fade != 0:
				newList.append(i)
		self.tagReadObjects = newList

