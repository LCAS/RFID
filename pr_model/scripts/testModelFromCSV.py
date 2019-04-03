#!/usr/bin/env python


"""
Tests gmm model
"""
import warnings
with warnings.catch_warnings():
    warnings.filterwarnings("ignore",category=DeprecationWarning)
    import sklearn


import time
import numpy as np
import pandas as pd
import sys
from sklearn.mixture import GMM
import cPickle as pickle

from sklearn.metrics import mean_squared_error
import matplotlib.mlab as ml
import matplotlib.pyplot as plt
from sklearn import mixture
from sklearn.preprocessing import StandardScaler
from sklearn import manifold
from sklearn.model_selection import train_test_split

def getMatrix(X, type):
    readings = X[X['use'] == type]
    # get test data and model, check that those samples have high probability
    # this gets a matrix without last column 'use'
    X0 = readings.as_matrix(columns=readings.columns[0:-1])
    return X0

def getScore(X, type, Gm, scal):
    X0 = getMatrix(X,type)

    # our gmm works on scaled features
    X_scaled = scal.transform(X0)
    score = Gm.score(X_scaled)
    return score


# Main function.
if __name__ == '__main__':
    # params
    if len(sys.argv)==1:
        modelURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20dB-Linda-FAB-LAB-V3_gmm_models.pickle'
        print 'WARNING! using default model pickle file!!!'
    else:
        fileURI = sys.argv[1]


    print 'modelFile: '+ modelURI

    # load data from pickle
    (freqs, gmm_list, aics_list, scaler_list, X0_list) = pickle.load(open(modelURI, "r"))

    index = 0

    freq = freqs[index]

    print('Testing model for freq ' + str((freq/1000.0)))

    gmm = gmm_list[index]
    aics = aics_list[index]
    scaler = scaler_list[index]
    df_X = X0_list[index]

    X_test = getMatrix(df_X,'test')
    X_training = getMatrix(df_X, 'training')

    X_test_scaled = scaler.transform(X_test)
    labels_test = gmm.predict(X_test_scaled)
    X_test_means = gmm.means_[labels_test]

    test_score = getScore(df_X,'test',gmm,scaler)
    train_score = getScore(df_X, 'training', gmm, scaler)



    rel_x_m = readings['rel_x_m']
    rel_y_m = readings['rel_y_m']
    rel_yaw_rad = readings['rel_yaw_rad']
    rssi_dbm = readings['rssi_dbm']
    phase_deg = readings['phase_deg']


    # create grid data
    grid_width = 100
    grid_height = 100
    num_x = 100
    num_y = 100
    levels = np.linspace(rssi_dbm.min(), rssi_dbm.max(), 20)
    x = np.linspace(rel_x_m.min(), rel_x_m.max(), num_x)
    y = np.linspace(rel_y_m.min(), rel_y_m.max(), num_y)


    # plot by linear interpolation of current data
    z = ml.griddata(rel_x_m, rel_y_m, rssi_dbm, x, y,interp='linear')
    c = plt.contourf(x, y, z, levels, alpha=0.5)

    plt.figure()
    Xg = scaler.inverse_transform(gmm.sample(samples))
    readings = pd.DataFrame(Xg, columns=header)

    rel_x_m = readings['rel_x_m']
    rel_y_m = readings['rel_y_m']
    rel_yaw_rad = readings['rel_yaw_rad']
    rssi_dbm = readings['rssi_dbm']
    phase_deg = readings['phase_deg']

    # create grid data
    grid_width = 100
    grid_height = 100
    num_x = 100
    num_y = 100
    levels = np.linspace(rssi_dbm.min(), rssi_dbm.max(), 20)
    x = np.linspace(rel_x_m.min(), rel_x_m.max(), num_x)
    y = np.linspace(rel_y_m.min(), rel_y_m.max(), num_y)

    # plot by linear interpolation of current data
    z = ml.griddata(rel_x_m, rel_y_m, rssi_dbm, x, y, interp='linear')
    c = plt.contourf(x, y, z, levels, alpha=0.5)




    #X0_test = X0[0:50, :]
    #(samples, features) = X0_test.shape
    #X0_test_sc = scaler.transform(X0_test)
    #print('samples ' + str(samples))
    #print('using ' + str(features)) + ' features: ' + ', '.join(header)


    scaler.inverse_transform(gmm.sample(1))

    # could be interesting
    #from scipy.interpolate import Rbf
    #rbfi = Rbf(rel_x_m ,    rel_y_m ,    rssi_dbm )






