#!/usr/bin/env python

import numpy as np
import pandas as pd
import math
from numpy.linalg import inv

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

def ensureTwoEntries(readings, index_x, index_y, index_a, index_f, xv, yv, av, fv):
    isFinished = False
    offset = 0
    while not isFinished:
        (xi,xi_prev) = getElements(xv,index_x+offset,index_x-offset-1,-np.inf,np.inf)
        (yi,yi_prev) = getElements(yv,index_y+offset,index_y-offset-1,-np.inf,np.inf)
        (ai,ai_prev) = getElements(av,index_a+offset,index_a-offset-1,-np.pi,np.pi)
        (fi,fi_prev) = getElements(fv,index_f+offset,index_f-offset-1,fv[0],fv[-1])

        subSet = readings[(readings['rel_x_m'] <= xi) & (readings['rel_x_m'] > xi_prev) &
                          (readings['rel_y_m'] <= yi) & (readings['rel_y_m'] > yi_prev) &
                          (readings['rel_yaw_rad'] <= ai) & (readings['rel_yaw_rad'] > ai_prev) &
                          (readings['freq_khz'] <= fi) & (readings['freq_khz'] > fi_prev)]

        if (subSet.size != 0) & (subSet['rssi_dbm'].size > 1):
            #Ok, enough data to even have covariance...

            rssi_i = subSet['rssi_dbm']
            phase_i = subSet['phase_deg']
            rssi_m = rssi_i.mean()
            phase_m = phase_i.mean()

            # compute covariance matrix and determinant
            X = np.stack((rssi_i, phase_i), axis=0)
            COV = np.cov(X)
            detC = np.linalg.det(COV)
            # we don't want singular, indefinde matrixes
            if (detC>0):
                # we don't want  definide negative covariance matrixes ...
                try:
                    invC = inv(COV)
                    isFinished = True
                    print offset
                except:
                    pass
        offset = offset+1

    return (xi, yi, ai, fi,rssi_m,phase_m,COV)

def ensureOneEntry(readings, index_x, index_y, index_a, index_f, xv, yv, av, fv):
    offset = 0
    isOk = False
    (xi,xi_prev) = getElements(xv,index_x+offset,index_x-offset-1,-np.inf,np.inf)
    (yi,yi_prev) = getElements(yv,index_y+offset,index_y-offset-1,-np.inf,np.inf)
    (ai,ai_prev) = getElements(av,index_a+offset,index_a-offset-1,-np.pi,np.pi)
    (fi,fi_prev) = getElements(fv,index_f+offset,index_f-offset-1,fv[0],fv[-1])

    subSet = readings[(readings['rel_x_m'] <= xi) & (readings['rel_x_m'] > xi_prev) &
                      (readings['rel_y_m'] <= yi) & (readings['rel_y_m'] > yi_prev) &
                      (readings['rel_yaw_rad'] <= ai) & (readings['rel_yaw_rad'] > ai_prev) &
                      (readings['freq_khz'] <= fi) & (readings['freq_khz'] > fi_prev)]

    if (subSet.size != 0) & (subSet['rssi_dbm'].size > 1):
        #Ok, enough data to even have covariance...

        rssi_i = subSet['rssi_dbm']
        phase_i = subSet['phase_deg']
        rssi_m = rssi_i.mean()
        phase_m = phase_i.mean()

        # compute covariance matrix and determinant
        X = np.stack((rssi_i, phase_i), axis=0)
        COV = np.cov(X)
        detC = np.linalg.det(COV)
        # we don't want singular, indefinde matrixes
        if (detC>0):
            # we don't want  definide negative covariance matrixes ...
            try:
                invC = inv(COV)
                isOk = True
                print offset
            except:
                pass
    if not isOk:
        rssi_m = -90
        phase_m = 0
        COV = np.diag([1, 1])

    return (xi, yi, ai, fi,rssi_m,phase_m,COV)


def getDiscreteCol(colName,df):
    return (df[colName]*100).astype('int')/100.0

def getLinspace(colName,df,samples):
    return np.linspace(df[colName].min(), df[colName].max(), num=samples)


def constrainAngle(x):
    x = math.fmod(x + np.pi,2.0*np.pi)
    if (x < 0):
        x = x + 2.0*np.pi
    return (x - np.pi)

    
# params
fileURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/30db/30db.csv'
modelURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/30db/30db_model.csv'

#fileURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/long/long_1tag.csv'
#modelURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/30db/long_1tag_model.csv'


selectedTag = '300833B2DDD9014000000014'
numSamples_x = 20
numSamples_y = 20
numSamples_a = 8

# load data from csv
# colnames 'Time','ID','rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm', 'phase_deg'
readings = pd.read_csv(fileURI)

# filter to get tag 14 (best one afaik)
#readings14 = readings[readings['ID'] == selectedTag].copy()

# discretize x,y,a values to for our 4-dimensional grid (f is already discrete)
readings['rel_x_m'] = getDiscreteCol('rel_x_m',readings)
readings['rel_y_m'] = getDiscreteCol('rel_y_m',readings)
readings['rel_yaw_rad'] = getDiscreteCol('rel_yaw_rad',readings)

# on each element of each dimension: calculate 2D gausian in terms of rssi AND phase 
xv = getLinspace('rel_x_m',readings,numSamples_x)
yv = getLinspace('rel_y_m',readings,numSamples_y)
av = getLinspace('rel_yaw_rad',readings,numSamples_a)
fv = np.sort(np.unique(readings['freq_khz']))

entryList=[]
for index_x in range(1,numSamples_x):
    for index_y in range(1,numSamples_y):
        for index_a in range(1,numSamples_a):
            for index_f in range(1,len(fv)):
                # this one increases cell size to fill in with data, but maybe it's better not to do that
                #(xi, yi, ai, fi, rssi_m, phase_m, COV) = ensureTwoEntries(readings,index_x,index_y,index_a,index_f,xv,yv,av,fv)
                (xi, yi, ai, fi, rssi_m, phase_m, COV) = ensureOneEntry(readings,index_x,index_y,index_a,index_f,xv,yv,av,fv)
                # x,v,a,f returned by ensureTwo are the real. These are the model ones
                xi = xv[index_x]
                yi = yv[index_y]
                ai = av[index_a]
                fi = fv[index_f]

                entry = (xi,yi,ai,fi,rssi_m,phase_m,COV[0,0],COV[0,1],COV[1,0],COV[1,1])
                entryList.append(entry)


labels = ['rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm_m', 'phase_deg_m','COV00','COV01','COV10','COV11']
daModel = pd.DataFrame.from_records(entryList, columns=labels)

print("Saving model to csv")
daModel.to_csv(modelURI, index=False)


