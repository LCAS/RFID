#!/usr/bin/env python

"""
Plots Average RSSI in an specific frequency of the given model

"""


from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import sys
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import math
import pickle

import scipy.ndimage

import matplotlib.colors as colors
import os
from nav_msgs.msg import  OccupancyGrid
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator

import scipy.ndimage
import pandas as pd

def getGridData(grid_vector):
    if isinstance(grid_vector, pd.core.series.Series):
        ranges = grid_vector.unique()
        gridSize = len(ranges)
        gridResolution = abs(ranges[1]-ranges[0])
        return (ranges,gridSize,gridResolution)
    else:
        print('getGridData: vector is not a pandas Series') 

# Main function.
if __name__ == '__main__':


    # constants
    transmissionPower_dBm = 20
    #origURI ='/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20db-linda-fab-lab.csv'
    #allData =  pd.read_csv(origURI)

    fileURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20db-linda-fab-lab_model.csv'
    #labels in csv should be ['rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm_m', 'phase_deg_m','COV00','COV01','COV10','COV11','count']    

    # ..........
    readings = pd.read_csv(fileURI)

    (ranges_a,gridSize_a,gridResolution_a) = getGridData(readings['rel_yaw_rad'])
    (ranges_f,gridSize_f,gridResolution_f) = getGridData(readings['freq_khz'])


    # lets pick best frequency and orientation:
    maxCount = 0
    for fi in ranges_f:
        for ai in ranges_a:
            subSet = readings[(readings['rel_yaw_rad'] == ai) & (readings['freq_khz'] == fi)]
            countI = subSet['count'].sum()
            #print countI
            if countI>maxCount:
                maxCount = countI
                aiF = ai
                fiF = fi
    subSet = readings[(readings['rel_yaw_rad'] == aiF) & (readings['freq_khz'] == fiF)]
   # print aiF*180.0/np.pi 
    #print fiF
   # print maxCount


    (ranges_x,gridSize_x,gridResolution_x) = getGridData(subSet['rel_x_m'])
    (ranges_y,gridSize_y,gridResolution_y) = getGridData(subSet['rel_y_m'])

    # pick the desired colormap, sensible levels, and define a normalization
    # instance which takes data values and translates those into levels.
    cmap = plt.get_cmap('OrRd')

    X, Y = np.mgrid[slice(ranges_x[0]-gridResolution_x, ranges_x[-1]+gridResolution_x, gridResolution_x),
                    slice(ranges_y[0]-gridResolution_y, ranges_y[-1]+gridResolution_y, gridResolution_y)]

    minX = ranges_x[0]
    maxX = ranges_x[-1]
    minY = ranges_y[0]
    maxY = ranges_y[-1]
    sig = 0.45

    width  = gridSize_x
    height = gridSize_y

    title=''

    avRssi = np.array(subSet['rssi_dbm_m']).reshape(width, height)
    detections = np.array(subSet['count']).reshape(width, height)

    avRssi = avRssi.astype(float)

    #avRssi[detections == 0] = -99


    reescaledRSSI = scipy.ndimage.filters.gaussian_filter(avRssi, sigma=sig, mode='reflect')
    reescaledRSSI = (reescaledRSSI + np.flipud(reescaledRSSI)) / 2
    reescaledRSSI = np.rot90(np.flipud(reescaledRSSI), 3)
    #reescaledRSSI = avRssi

    showAxis=True
    #fig, ax = plt.subplots(nrows=1, ncols=1)

    gs = gridspec.GridSpec(1, 2,
                           width_ratios=[10, 1],
                           )
    ax = plt.subplot(gs[0])
    ax2 = plt.subplot(gs[1])


    total = np.array(reescaledRSSI)
    #threshold=-74.4
    #total[(reescaledRSSI < threshold)] = 0


    ax.plot(-0, -0, "w>", markersize=8, label='Robot Pose')
    #ax.legend(loc='upper center', shadow=True)



    cf = ax.pcolormesh(X, Y, total, cmap=plt.get_cmap('bone'),edgecolor='grey',linewidth=0.01)
    #                      norm=colors.LogNorm(vmin=1, vmax=realMax - realMin + 1),
    #                      cmap=cmap,shading='none')
    cbar=plt.colorbar(cf, cax=ax2)#,ticks=[0, -8, -16, -24, -32, -40, -48, -56, -64])
    #cbar.ax.set_yticklabels(['< -4 dB', '-8', '-16', '-24', '-32', '-40', '-48', '-56', '-64'])

    #ax.legend(loc='upper center', shadow=True)
    ax.get_xaxis().set_visible(showAxis)
    ax.get_yaxis().set_visible(showAxis)
    ax.set_xlim((minX, maxX))
    ax.set_ylim((minY, maxY))
    ax.set_aspect('equal')
    ax.set_xlabel('X (m.)')
    ax.set_ylabel('Y (m.)')
    ax.set_title(title)
    plt.show()
    
    
    
    
    
