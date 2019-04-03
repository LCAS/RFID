#!/usr/bin/env python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


c =  3e8 #299792458.0
logC = 20.0 * np.log10( c )

def diffMatrix(v):
    '''

    :param v: np array with input values
    :return: triangular difference matrix a where a[i,j]= v[i]-v[j]
    '''
    n_reads = len(v)
    temp = np.triu(np.ones((n_reads, n_reads)), 1)
    Vj = temp * (v.reshape(1,n_reads))
    Vi = temp * (v.reshape(n_reads,1))
    diffM = Vi - Vj
    return diffM

# Main function.
if __name__ == '__main__':
    # params
    fileURI='/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20db-linda-fab-lab-Static-Data.csv'

    print 'dataFile: '+fileURI

    # load data from csv
    # colnames: Time,ID,rel_x_m,rel_y_m,rel_yaw_rad,freq_khz,rssi_dbm,phase_deg,robot_x_m,robot_y_m,robot_yaw_rad
    header = ['Time','ID', 'rel_r_m', 'rel_tetha_rad', 'rel_yaw_rad', 'rssi_dbm', 'phase_deg']
    allData0 = pd.read_csv(fileURI)

    # divide dataset per frequencies:
    freqs = np.sort(np.unique(allData0['freq_khz']))
    tags = np.unique(allData0['ID'])
    print ('dataset contains [%d] tags ' % len(tags))

    # some relevant features to choose from
    bestTag = allData0.groupby('ID').size().sort_values().index[-1]
    worstTag = allData0.groupby('ID').size().sort_values().index[0]
    bestFreq = allData0.groupby('freq_khz').size().sort_values().index[-1]

    # input data filter
    allData = allData0[allData0['ID'] == bestTag ]
    #allData = allData0[allData0['ID'] == worstTag]
    #allData = allData[allData['freq_khz'] == bestFreq]
    # allData = allData0

    # get readings
    readings = allData
    print("Selected %d points" % (readings.shape[0]))

    timestamps = readings['Time']
    freq_khz = readings['freq_khz']

    rel_x_m = readings['rel_x_m']
    rel_y_m = readings['rel_y_m']
    rel_yaw_rad = readings['rel_yaw_rad']
    rssi_dbm = readings['rssi_dbm']
    phase_deg = readings['phase_deg']

    # calculate r and rel_tetha_rad
    rel_r_m = np.sqrt(rel_x_m.values*rel_x_m.values + rel_y_m.values*rel_y_m.values)
    rel_tetha_rad = np.arctan2(rel_y_m.values, rel_x_m.values)

    freq_hz = freq_khz.values * 1000.0
    phase_rad =phase_deg.values  *  np.pi/180.0



    from sklearn import linear_model


    X = freq_hz.reshape(-1, 1)
    y = phase_rad.reshape(-1, 1)
    outlierFactor = 0.25
    madY=phase_deg.mad()* outlierFactor * np.pi/180.0
    # Robustly fit linear model with RANSAC algorithm
    ransac = linear_model.RANSACRegressor(residual_threshold=madY)
    ransac.fit(X, y)
    inlier_mask = ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)

    # selected points
    goodFreqs= X[inlier_mask]
    goodPhases=y[inlier_mask]

    # average them by frequency
    goodData = pd.DataFrame(data=np.hstack((goodFreqs,goodPhases)), columns=['freq', 'phase'])
    meanPhase = goodData.groupby('freq').mean()
    meanPhase.plot()

    plt.scatter(X[inlier_mask], y[inlier_mask], color='yellowgreen', marker='.',
                label='Inliers')
    plt.scatter(X[outlier_mask], y[outlier_mask], color='gold', marker='.',
                label='Outliers')


    # Get distance
    f_vector = meanPhase.index.values.flatten()
    ph_vector = meanPhase.values.flatten()

    deltaF = np.diff(f_vector)
    deltaph = np.diff(ph_vector)
    inc = deltaF / deltaph

    est_dist = -c / (4.0 * np.pi * inc)
    print("Estimated distance: %3.3f, std: %3.3f" % (est_dist.mean(),est_dist.std()))

