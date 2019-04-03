#!/usr/bin/env python

import time
import numpy as np
import pandas as pd
import sys
from sklearn.mixture import GMM
import cPickle as pickle
import matplotlib.pyplot as plt


import warnings
with warnings.catch_warnings():
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    import sklearn
    from sklearn import mixture
    from sklearn.preprocessing import StandardScaler
    from sklearn import manifold
    from sklearn.model_selection import train_test_split

"""
Reads csv file created with parseRosbagsNoPlay.py and creates a gmm model
where features are:
 tag-antenna 2D polar coordinates, orientation,freq, received power and received phase
"""


# Main function.
if __name__ == '__main__':
    # params
    if len(sys.argv) == 1:
        fileURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20dB-Linda-FAB-LAB-V3.csv'
        print 'WARNING! using default dataFile!!!'
    else:
        fileURI = sys.argv[1]

    modelURI = fileURI[0:-4]+'_gmm_model_single_polar.pickle'
    minmodelURI = fileURI[0:-4] + '_gmm_model_single_min_polar.pickle'
    print 'dataFile: '+fileURI
    print 'modelFile: '+modelURI

    # load data from csv
    # colnames:  Time       rel_x_m       rel_y_m   rel_yaw_rad       freq_khz      rssi_dbm     phase_deg     robot_x_m     robot_y_m  robot_yaw_rad
    header = ['Time', 'rel_r_m', 'rel_tetha_rad',
              'rel_yaw_rad', 'freq_khz', 'rssi_dbm', 'phase_deg']
    allData = pd.read_csv(fileURI)

    print('Creating  model')
    start_time = time.time()
    init_time = start_time
    print('Data processing starting at ' + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(start_time)))
    print '\n '

    readings = allData
    timestamps = readings['Time']
    rel_x_m = readings['rel_x_m']
    rel_y_m = readings['rel_y_m']

    # calculate r and rel_tetha_rad
    rel_r_m = np.sqrt(rel_x_m.values*rel_x_m.values + rel_y_m.values*rel_y_m.values)
    rel_tetha_rad = np.arctan2(rel_y_m.values, rel_x_m.values)

    rel_yaw_rad = readings['rel_yaw_rad']
    freq_khz = readings['freq_khz']
    rssi_dbm = readings['rssi_dbm']
    phase_deg = readings['phase_deg']

    X0 = np.stack((timestamps, rel_r_m, rel_tetha_rad,
                   rel_yaw_rad, freq_khz, rssi_dbm, phase_deg), axis=1)
    (samples, features) = X0.shape
    features = features - 1  # time wont go into model
    # creating model
    print '....................................................................... '
    print('samples '+str(samples))
    print('features ' + str(features))

    # divide dataset between train and test
    (X_train, X_test) = train_test_split(X0)

    # do not use time from now on. But still useful as an index
    t = X0[:, 0]
    t_train = X_train[:, 0]
    t_test = X_test[:, 0]
    X_train = X_train[:, 1:]
    X_test = X_test[:, 1:]
    X0 = X0[:, 1:]

    # dataframe vodooo!
    df_X_train = pd.DataFrame(data=X_train, index=t_train, columns=header[1:])
    df_X_train['use'] = 'training'
    df_X_test = pd.DataFrame(data=X_test, index=t_test, columns=header[1:])
    df_X_test['use'] = 'test'
    df_X = pd.concat([df_X_test, df_X_train])
    df_X['use'] = df_X.use.astype('category')
    df_X.sort_index(inplace=True)

    # scale data to prevent bias
    # Note that scaler is defined acording to the whole dataset!
    scaler = StandardScaler()
    scaler = scaler.fit(X0)

    X_scaled = scaler.transform(X_train)

    # find best gmm model aics criterium
    n_components = np.arange(5, 200, 5)
    models = [GMM(n, covariance_type='full') for n in n_components]
    aics = [model.fit(X_scaled).aic(X_scaled) for model in models]
    selected_model = np.argmin(aics, axis=0)

    print('Best model has ' + str(n_components[selected_model]) + ' components')
    print('features ' + str(features))

    gmm = models[selected_model]
    aics = aics[selected_model]
    end_time = time.time()

    print('It lasted ' + str(end_time-start_time) + ' secs')
    print('\n')
    start_time = end_time

    end_time = time.time()
    print('In total, it lasted ' + str((end_time-init_time)/60.0) + ' mins')
    print('\n')
    start_time = end_time
    saved_data = (gmm, aics, scaler, df_X)
    pickle.dump(saved_data, open(modelURI, "wb"))

    min_saved_data = (gmm, scaler)
    pickle.dump(min_saved_data, open(minmodelURI, "wb"))
