#!/usr/bin/env python


"""
unfinished, do not use

"""

import numpy as np
import pandas as pd
import math
from numpy.linalg import inv
import sys

def getElements(v,indexH,indexL,defaultLow,defaultHigh):
    if (indexH<len(v))&(indexH>=0):
        vi = v[indexH]
    else:
        vi = defaultHigh

    if (indexL<len(v))&(indexL>=0):
        vip = v[indexL]
    else:
        vip = defaultLow

    return (vi,vip)



def getDiscreteCol(colName,df):
    return (df[colName]*100).astype('int')/100.0

def getLinspace(colName,df,samples):
    return np.linspace(df[colName].min(), df[colName].max(), num=samples)


def constrainAngle(x):
    x = math.fmod(x + np.pi,2.0*np.pi)
    if (x < 0):
        x = x + 2.0*np.pi
    return (x - np.pi)

# Main function.
if __name__ == '__main__':    
    # params
    if len(sys.argv)==1:
        #fileURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/long/long_1tag.csv'
        fileURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/30db/30db.csv'
        print 'WARNING! using default dataFile!!!'
    else:
        fileURI = sys.argv[1]
  
    modelURI = fileURI[0:-4]+'_model.csv'
    print 'dataFile: '+fileURI
    print 'modelFile: '+modelURI

    numSamples_x = 20
    numSamples_y = 20
    numSamples_a = 20
    numSamples_f = np.unique(readings['freq_khz']).size
    # load data from csv
    # colnames 'Time','ID','rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm', 'phase_deg'
    readings = pd.read_csv(fileURI)
    dataMatrix = np.loadtxt(open(origURI, "rb"), delimiter=",", skiprows=1)
    # dataMatrix [entry, field]
    # dataMatrix[:,field]
    
    #rssi matrix has as indexes ['rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', ]
    rssiMatrix = a = np.ndarray(shape=(numSamples_x,numSamples_y,numSamples_a,numSamples_f),dtype =list())

    # this is our model 4D grid
    xv = getLinspace('rel_x_m',readings,numSamples_x)
    yv = getLinspace('rel_y_m',readings,numSamples_y)
    av = getLinspace('rel_yaw_rad',readings,numSamples_a)
    fv = np.sort(np.unique(readings['freq_khz']))

    entryList=[]
    # on each element of each dimension: calculate 2D gausian in terms of rssi AND phase 
    for index_x in range(1,numSamples_x):
        for index_y in range(1,numSamples_y):
            for index_a in range(1,numSamples_a):
                for index_f in range(1,len(fv)):
                    # this one increases cell size to fill in with data, but maybe it's better not to do that
                    #(xi, yi, ai, fi, rssi_m, phase_m, COV) = ensureTwoEntries(readings,index_x,index_y,index_a,index_f,xv,yv,av,fv)
                    (xi, yi, ai, fi, rssi_m, phase_m, COV,count) = ensureOneEntry(readings,index_x,index_y,index_a,index_f,xv,yv,av,fv)
                    # x,v,a,f returned by ensureTwo are the real. These are the model ones
                    xi = xv[index_x]
                    yi = yv[index_y]
                    ai = av[index_a]
                    fi = fv[index_f]
                    
                    entry = (xi,yi,ai,fi,rssi_m,phase_m,COV[0,0],COV[0,1],COV[1,0],COV[1,1],count)
                    entryList.append(entry)
                    print '.'

    labels = ['rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm_m', 'phase_deg_m','COV00','COV01','COV10','COV11','entries']
    daModel = pd.DataFrame.from_records(entryList, columns=labels)

    print("Saving model to csv")
    daModel.to_csv(modelURI, index=False)


