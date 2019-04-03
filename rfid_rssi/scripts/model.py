#!/usr/bin/env python

#import ros
import rospy
import tf

#for loading models
import pickle
import os

#getting command line arguments
import sys

#for reading rfid messages
from std_msgs.msg import String

#for writing navigation og messages
from nav_msgs.msg import *

#for trig for map calculations
import math

#Configurables - distances in meters
#gridSize = distance from robot to edge of map in all directions
#gridResolution = the cell size of the map
gridSize = 9
gridResolution = 0.3
transmissionPower = 3000

#globals
tagIDs = []
tfListener = ""
gridTotal = int(gridSize/gridResolution)
#model, each element is a frequency model
#each frequency model has the following format
#[ROS publisher, frequency(kHz), 2D ROS occupancy grid,
#detections, mean, standard deviation]
model = []

def tagCallback(data):

	data = str(data)
	data = data.split(":")
	tagID = data[2]
	freq = data[5]
	rssi = int(data[3])
	tx = 3000

	if not tagID in tagIDs:
		return
 	if not tfListener.frameExists("/base_link"):
		print "Unable to find tf frame for base link"
		return
	if not tfListener.frameExists("/map"):
		print "Unable to find tf frame for map"
		return
	if not tfListener.frameExists("/rfid/tags/" + tagID):
		print "Unable to find tf frame for rfid tag: " + tagIDs[0]
		return

	now = rospy.Time(0)
	tfListener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))
	r_pos, r_quat = tfListener.lookupTransform("/map", "/base_link", now)
	tfListener.waitForTransform("map", "rfid/tags/" + tagID, now, rospy.Duration(4.0))
	t_pos, t_quat = tfListener.lookupTransform("/map", "/rfid/tags/" + tagID, now)

	foundFrequency = False

	for i in model:
		if i[1] == freq:
			foundFrequency = True

	if foundFrequency == False:
		createModel(freq)

	for i in model:
		if i[1] == freq:
			roll, pitch, yaw = tf.transformations.euler_from_quaternion([r_quat[0], r_quat[1], r_quat[2], r_quat[3]])

			xt = t_pos[0] - r_pos[0]
			yt = t_pos[1] - r_pos[1]
			x = xt * math.cos(-yaw) - yt * math.sin(-yaw)
			y = xt * math.sin(-yaw) + yt * math.cos(-yaw)
			z = 0

			if x > gridTotal or y > gridTotal or -x > gridTotal or -y > gridTotal:
				print "Warning: Tag detected beyond the range of the model being created"
				return

			yoffset = ((y - (y % gridResolution)) + gridSize) / gridResolution
			xoffset = ((x - (x % gridResolution)) + gridSize) / gridResolution
			cellIndex = int((gridTotal * 2 * yoffset) + xoffset)

			#update freq-specific model

			n = i[3][cellIndex]
			mean = i[4][cellIndex]
			rt = i[6][cellIndex]
			rt2 = i[7][cellIndex]

			n, rt, rt2, mean, sd = recursiveSD(rssi, n, rt, rt2, mean)

			i[3][cellIndex] = n
			i[4][cellIndex] = mean
			i[5][cellIndex] = sd
			i[6][cellIndex] = rt
			i[7][cellIndex] = rt2

			#update combined model

			n = model[0][3][cellIndex]
			mean = model[0][4][cellIndex]
			rt = model[0][6][cellIndex]
			rt2 = model[0][7][cellIndex]

			n, rt, rt2, mean, sd = recursiveSD(rssi, n, rt, rt2, mean)

			model[0][3][cellIndex] = n
			model[0][4][cellIndex] = mean
			model[0][5][cellIndex] = sd
			model[0][6][cellIndex] = rt
			model[0][7][cellIndex] = rt2

			#good approximation for mapping rssi to 0-100 scale
			display_mean = (i[4][cellIndex] * -2) - 60
			display_combinedMean = (model[0][4][cellIndex] * -2) - 60

			i[2].data[cellIndex] = display_mean
			model[0][2].data[cellIndex] = display_combinedMean
			
			i[0].publish(i[2])
			model[0][0].publish(model[0][2])

def recursiveSD(val, n, rt, rt2, mean):
	if n == 0:
		n = 1
		rt = val
		rt2 = val * val
		mean = val
		sd = None
	else:
		n += 1
		rt = rt + val
		rt2 = rt2 + (val * val)
		mean = ((mean * (n - 1)) + val) / n
		sd = math.sqrt((n * rt2 - rt * rt)/(n * (n - 1)))
	return n, rt, rt2, mean, sd

def createModel(freq):
	global model

	map = createMap()
	detections = []
	mean = []
	sd = []
	runningTotal = []
	runningTotal2 = []

	for i in xrange((gridTotal * 2) * (gridTotal * 2)):
		detections.append(0)
		mean.append(None)
		sd.append(None)
		runningTotal.append(None)
		runningTotal2.append(None)

	pub = rospy.Publisher("rfid/sensor_model/" + str(freq), OccupancyGrid, queue_size=10)

	newFreq = [pub, freq, map, detections, mean, sd, runningTotal, runningTotal2]
	model.append(newFreq)

def createMap():
	map = OccupancyGrid()
	map.header.frame_id = "/base_link"
	map.info.resolution = gridResolution
	map.info.width = gridTotal * 2
	map.info.height = gridTotal * 2
	map.info.origin.position.x = -gridSize
	map.info.origin.position.y = -gridSize
	map.info.origin.position.z = 0
	map.info.origin.orientation.w = 1
	map.info.origin.orientation.x = 0
	map.info.origin.orientation.y = 0
	map.info.origin.orientation.z = 0
	map.data = []
	for i in xrange(0, (gridTotal * 2) * (gridTotal * 2)):
		map.data.append(-1)
	return map

def recoverModel():

	fname = "models/" + str(transmissionPower) + ".p"

	if os.path.isfile(fname):
		model = pickle.load(open(fname, "rb"))

	for i in model:
		i[0] = rospy.Publisher("rfid/sensor_model/" + str(freq), OccupancyGrid, queue_size=10)

def dumpModel():

	fname = "models/" + str(transmissionPower) + ".p"

	for i in model:
		i[0] = ""

	pickle.dump(model, open(fname, "wb"))

def main():

	global tfListener, model
	
	args = sys.argv
	del args[0]

	if len(args) == 0:
		print "Error: No tags to build model from"
		return

	if "-a" in args:
		recoverModel()
		args = args.remove("-a")
	else:
		createModel("combined")
		model[0].append(gridSize)
		model[0].append(gridResolution)

	for i in args:
		tagIDs.append(i)

	rospy.init_node("buildModel")
	tfListener = tf.TransformListener()
	rfidSub = rospy.Subscriber("rfid/rfid_detect", String, tagCallback)

	rospy.spin()

	dumpModel()

if __name__ == "__main__":
	main()

