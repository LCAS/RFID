#!/usr/bin/env python
import time
import numpy as np
import pandas as pd
import sys
from sklearn.mixture import GaussianMixture
import cPickle as pickle
import matplotlib.pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF, Matern, RationalQuadratic,
                                              ExpSineSquared, DotProduct,
                                              ConstantKernel)
import warnings

"""
Reads csv file created with parseRosbagsNoPlay.py and tests friis equation

"""


c =  3e8 #299792458.0
logC = 20.0 * np.log10( c )

gainDict = {
            '-90': -20,
            '-85': -17.56756757,
            '-80': -12.16216216,
            '-75': -11.62162162,
            '-70': -10,
            '-65': -8.648648649,
            '-60': -8.108108108,
            '-55': -6.216216216,
            '-50': -5.405405405,
            '-45': -4.594594595,
            '-40': -2.972972973,
            '-35': -2.432432432,
            '-30': -2.162162162,
            '-25': -1.891891892,
            '-20': -1.621621622,
            '-15': -0.8108108108,
            '-10': -0.5405405405,
            '-5': -0.2702702703,
            '0': 0,
            '5': -0.2702702703,
            '10': -0.5405405405,
            '15': -0.8108108108,
            '20': -1.621621622,
            '25': -1.891891892,
            '30': -2.162162162,
            '35': -2.432432432,
            '40': -2.972972973,
            '45': -4.594594595,
            '50': -5.405405405,
            '55': -6.216216216,
            '60': -8.108108108,
            '65': -8.648648649,
            '70': -10,
            '75': -11.62162162,
            '80': -12.16216216,
            '85': -17.56756757,
            '90': -20}


def antennasGain(t):
    global gainDict
    adeg0 = t * 180.0 / np.pi
    adegRound = (5 * np.round(adeg0 / 5)).astype(int).astype(str)

    ans=[]
    for a in adegRound:
        try:
            totalGain = gainDict[a]
        except KeyError:
            totalGain = -50.0
        ans.append(totalGain)

    return np.array(ans)

# propagation losses due to frequency
def waveLenFactor(f):
    global logC
    loss= logC - 20.0 * np.log10(f)
    return loss

# phase delay due to traveled distance
def propagatedWaveDelay(r,f):
    phi = (4.0 * np.pi * r *f / c ) % (2.0*np.pi)
    return phi

# propagation losses due to distance in free space
def distFactor(r):
    return 20.0 * np.log10(4.0 * np.pi * r)

'''
Pt(dBm) = Pr(dBm) – Gt – Gr + 20*log(4πd/λ)
Pt(dBm) = Pr(dBm) – Gt – Gr + 20*log(4πdf/c)
Pr(dBm) = Pt(dBm) + Gt + Gr - 20*log(4πdf/c)
Pr(dBm) = Pt(dBm) + Gt + Gr - 20*log(4π/c) - 20*log(df)
Pr(dBm) = Pt(dBm) + Gt + Gr + 20*log(c/4π) - 20*log(df)
'''


def friisEq(r,th,f,txp_dbm):
    rxp_dbm = txp_dbm + antennasGain(th) + waveLenFactor(f) - distFactor(r)
    phase = propagatedWaveDelay(r,f)
    return (rxp_dbm,phase)


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
    if len(sys.argv) == 1:
        fileURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20dB-Linda-FAB-LAB-V3.csv'
        print 'WARNING! using default dataFile!!!'
    else:
        fileURI = sys.argv[1]

    fileURI='/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20db-linda-fab-lab-Static-Data.csv'

    modelURI = fileURI[0:-4]+'_gmm_models_polar_label.pickle'
    minmodelURI = fileURI[0:-4] + '_gmm_models_min_polar_label.pickle'
    print 'dataFile: '+fileURI
    print 'modelFile: '+modelURI

    # load data from csv
    # colnames: Time,ID,rel_x_m,rel_y_m,rel_yaw_rad,freq_khz,rssi_dbm,phase_deg,robot_x_m,robot_y_m,robot_yaw_rad
    header = ['Time','ID', 'rel_r_m', 'rel_tetha_rad', 'rel_yaw_rad', 'rssi_dbm', 'phase_deg']
    allData0 = pd.read_csv(fileURI)

    # divide dataset per frequencies:
    freqs = np.sort(np.unique(allData0['freq_khz']))
    tags = np.unique(allData0['ID'])
    print ('dataset contains [%d] tags ' % len(tags))

    # all tags are the same, but let's build a model specific to one tag
    bestTag = allData0.groupby('ID').size().sort_values().index[-1]
    worstTag = allData0.groupby('ID').size().sort_values().index[0]

    allData = allData0[allData0['ID'] == bestTag ]
    #allData = allData0[allData0['ID'] == worstTag]
    #allData = allData0

    bestFreq = allData.groupby('freq_khz').size().sort_values().index[-1]
    allData = allData[allData['freq_khz'] == bestFreq]

    readings = allData
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









    # try estimating distance with phase

    fstats = allData.groupby('freq_khz').mean()
    f_vector = fstats.index.values * 1000.0
    ph_vector = fstats.phase_deg.values*  np.pi/180.0
    deltaF = diffMatrix(f_vector)
    deltaph = diffMatrix(ph_vector)
    deltaph[deltaph>0]=np.nan
    deltaph[deltaph > 0] = np.nan
    inc= deltaF.flatten()/deltaph.flatten()
    inc=inc[np.isfinite(inc)]
    # deltaF = np.diff(f_vector)
    # deltaph = np.diff(ph_vector)
    # inc= deltaF/deltaph
    est_dist = c/(4.0*np.pi*inc)
    plt.plot(est_dist)






    freqs = freq_khz.values * 1000.0
    f = freqs[freqs<9.09e8]
    ph =phase_deg[freqs<9.09e8]  *  np.pi/180.0
    plt.plot(f, ph, 'rx')

    plt.plot(freqs, phase_deg*np.pi/180.0, 'xr')
    fstats = allData.groupby('freq_khz').mean()
    plt.plot(fstats.index.values * 1000.0, fstats.phase_deg.values*  np.pi/180.0   )
    plt.plot(fstats.phase_deg.values * np.pi / 180.0)
    # use autocorr to get period
    from statsmodels import api as sm

    freqs = freq_khz.values * 1000.0
    ph = propagatedWaveDelay(rel_r_m, freqs)
    plt.plot(freqs, ph, '.')
    plt.plot(freqs, phase_deg*np.pi/180.0, '.')

    # t=rel_yaw_rad.values
    # t[t>np.pi/2]= np.pi - t[t>np.pi/2]
    # t[t < -np.pi / 2] = np.pi + t[t < -np.pi / 2]
    # plt.plot(t, rssi_dbm, '.')

    # https://en.wikipedia.org/wiki/Log-distance_path_loss_model
    deltaP=diffMatrix(rssi_dbm.values)
    # https://en.wikipedia.org/wiki/ITU_model_for_indoor_attenuation
    deltaLogDist=diffMatrix(33.0* np.log10( rel_r_m) )

    Xg=(deltaP-deltaLogDist).flatten()
    hist, bin_edges = np.histogram(Xg, density=True)
    plt.hist(Xg, bins=bin_edges, normed=True)

    # we have a 180 deg simetry
    t=rel_yaw_rad.values
    t[t>np.pi/2]= np.pi - t[t>np.pi/2]
    t[t < -np.pi / 2] = np.pi + t[t < -np.pi / 2]

    G=antennasGain(a)
    deltaG=diffMatrix(G)
    Xg2=(deltaP-deltaLogDist-deltaG).flatten()
    hist, bin_edges = np.histogram(Xg2, density=True)
    plt.hist(Xg2, bins=bin_edges, normed=True)


    expected_rssi = friisEq(rel_r_m,rel_tetha_rad, np.array(freq_khz)*1000.0,20.0)

