#!/usr/bin/env python

'''

Loads a pickle model and fills in grid cells without data using surrounding data

'''



from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import matplotlib.pyplot as plt
import math
import pickle

import matplotlib.colors as colors
import os
from nav_msgs.msg import  OccupancyGrid
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator
import scipy.ndimage
import logging


def interpolateModel(m,aGm):
    sig=0.6
    mod='reflect'

    width = m[2].info.width
    height = m[2].info.height

    new_m=[]

    try:
        f2 = str(float(m[1]) / 1000)  # won't work with average
    except ValueError:
        f2 = m[1]

    logging.info('Interpolating Model at freq %s',str(f2))



    avRssi = np.array(m[4]).reshape(width, height)

    detections = np.array(m[3]).reshape(width, height)

    maxR=np.max(avRssi)
    minR=np.min(avRssi[detections!=0])


    countMatrix=np.zeros((width, height))
    countMatrix[detections==0]=1
    missingPoints=np.sum(countMatrix)
    totalReadings = np.sum(detections)
    totalPoints=(width*height)
    dataPoints=totalPoints-missingPoints
    emptyPercent=missingPoints/totalPoints

    numReadings=np.sum(detections)

    logging.info('-%% of empty data cells: %s',str(emptyPercent))
    logging.info('-# of readings: %s', str(numReadings))
    logging.info('-av # of readings per cell (with data): %s', str(totalReadings/dataPoints))
    logging.info(' ')

    #avRssi = avRssi.astype(float)


    #reescaledAVR = scipy.ndimage.filters.gaussian_filter(avRssi, sigma=sig,mode=mod)


    return (maxR,minR)
#/////////////////////////////


# Main function.
if __name__ == '__main__':
    gridSize = 9
    gridResolution = 0.3
    transmissionPower = 3000

    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

    #print os.getcwd()
    try:
        fname = "../models/" + str(transmissionPower) + ".p"
        file = (open(fname, "rb"))
    except IOError:
        fname = "scripts/models/" + str(transmissionPower) + ".p"
        file = (open(fname, "rb"))

    modelList = pickle.load(file)
    modelList = sorted(modelList, key=lambda m: m[1])
    aggregatedModel=modelList[-1]
    new_modelList=[]
    allRs=[]
    for m in modelList:
        (maxR, minR)=interpolateModel(m,aggregatedModel)
        allRs.append(maxR)
        allRs.append(minR)

    allRs = np.array(allRs)
    maxR = np.max(allRs)
    minR = np.min(allRs)

    print allRs
    print "........."
    print maxR
    print minR
    print "........."