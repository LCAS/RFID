import tf
import threading
import Queue
import time

from TagState import *
from SpatioModel import SpatioModel

class ParticleFilter(threading.Thread):
	
	def __init__(self, exitQueue):

		threading.Thread.__init__(self)

		self.exitQueue = exitQueue

		self.tagStates = []

		self.model = SpatioModel("models/3000.p")

		self.tf = tf.TransformListener()

	def run(self):

		try:
			msg = self.exitQueue.get(False)

			if msg == "exit":
				return

			else:

				(id, rssi, phase, freq) = msg
				now = rospy.Time(0)
				self.tf.waitForTransform("/map", "/base_link", now, rospy.Duration(2.0))
				pos, orientation = self.tf.lookupTransform("/map", "/base_link", now)
				newTagRead = TagRead(id, rssi, phase, freq, pos, orientation, time.time())

				foundTag = False

				for i in self.tagStates:
					if i.id == msg.id:
						foundTag = True
						i.append(msg)
				if foundTag == False:
					tagState = TagState(msg.id)
					tagState.append(msg)
					self.tagStates(tagState)

		except Queue.Empty:
			pass

		
		for i in self.tagStates:

			i.fade(60)
			i.updateParticles(self.tf, self.model)

			
