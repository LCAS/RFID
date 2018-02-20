#!/usr/bin/env python
import ros, tf
import random, time
import threading, Queue

from TagState import *
from ParticleFilter import ParticleFilter

PARTICLE_COUNT = 1000

tagStates = []
threadQueue = Queue.Queue()

def parseTagData(data):
	data = data.split(":")
	id = data[1]
	rssi = int(data[2])
	phase = data[3]
	freq = data[4]
	return (id, rssi, phase, freq)

def tagCallback(data):

	msg = parseTagData(data)
	threadQueue.put(msg)

def main():

	random.seed(time.time())

	threadQueue = Queue.Queue()
	particleThread = ParticleFilter(threadQueue)

	rospy.init_node('particle_filter_f')
	rospy.Subscriber("rfid/rfid_detect", String, tagCallback)

	rospy.spin()

	threadQueue.put("exit")

if __name__ == '__main__':
	main()
