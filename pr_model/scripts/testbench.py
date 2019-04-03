#!/usr/bin/env python
import time
import numpy as np
import pandas as pd
import sys
from sklearn.mixture import GaussianMixture

try:
    #python 3
    import _pickle as pickle
except:
    #python 2.7
    import cPickle as pickle

import matplotlib.pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF, Matern, RationalQuadratic,
                                              ExpSineSquared, DotProduct,
                                              ConstantKernel, ExpSineSquared, WhiteKernel)
import warnings
with warnings.catch_warnings():
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    import sklearn
    from sklearn import mixture
    from sklearn.preprocessing import StandardScaler
    from sklearn import manifold
    from sklearn.model_selection import train_test_split

"""
Reads csv file created with parseRosbagsNoPlay.py and creates a gmm model per freq
where features are:
 tag-antenna 2D polar coordinates, orientation, received power and received phase


"""


def createDataframe(xtrain, ttrain, xtest, ttest, colnames):
    df_X_train = pd.DataFrame(data=xtrain, index=ttrain, columns=colnames)
    df_X_train['use'] = 'training'
    df_X_test = pd.DataFrame(data=xtest, index=ttest, columns=colnames)
    df_X_test['use'] = 'test'
    df_X = pd.concat([df_X_test, df_X_train])
    df_X['use'] = df_X.use.astype('category')
    df_X.sort_index(inplace=True)
    return df_X


# Main function.
if __name__ == '__main__':
    # params
    if len(sys.argv) == 1:
        fileURI = '/home/manuel/20dB-Linda-FAB-LAB-V3.csv'
        fileURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20dB-Linda-FAB-LAB-V3.csv'

        print('WARNING! using default dataFile!!!')
    else:
        fileURI = sys.argv[1]

    modelURI = fileURI[0:-4]+'_gmm_models_polar_label.pickle'
    minmodelURI = fileURI[0:-4] + '_gmm_models_min_polar_label.pickle'
    print ('dataFile: '+fileURI)
    print ('modelFile: '+modelURI)

    # load data from csv
    # colnames: Time,ID,rel_x_m,rel_y_m,rel_yaw_rad,freq_khz,rssi_dbm,phase_deg,robot_x_m,robot_y_m,robot_yaw_rad
    header = ['Time', 'ID', 'rel_r_m', 'rel_tetha_rad', 'rel_yaw_rad', 'rssi_dbm', 'phase_deg']
    allData0 = pd.read_csv(fileURI)

    # divide dataset per frequencies:
    freqs = np.sort(np.unique(allData0['freq_khz']))
    tags = np.unique(allData0['ID'])

    # all tags are the same, but let's build a model specific to one tag
    bestTag = allData0.groupby('ID').size().sort_values().index[-1]
    allData = allData0[allData0['ID'] == bestTag]

    print('Creating  ' + str(len(freqs)) + ' frequency models for tag '+bestTag)
    start_time = time.time()
    last_time = start_time
    print('Data processing starting at ' + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(start_time)))
    print ('\n ')
    gmm_list = []
    aics_list = []
    scaler_list = []
    df_X_list = []

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


                            #0          1           2          3          4          5          6
    dataset = np.stack((timestamps, freq_khz, rel_r_m, rel_tetha_rad, rel_yaw_rad, rssi_dbm, phase_deg), axis=1)
    (samples, features) = dataset.shape
    features = features - 3  # time wont go into model, rssi and phase are label

    # divide dataset between train and test
    (dataset_train, dataset_test) = train_test_split(dataset)

    # do not use time from now on. But still useful as an index

    # time, not in model
    t_train = dataset_train[:, 0]
    t_test = dataset_test[:, 0]
    t = dataset[:, 0]

    # freq_khz and phase_deg  are samples
    samples_indexs=(1,6)
    samples_header = ['freq_khz', 'phase_deg']
    X_train = dataset_train[:,samples_indexs]
    X_test = dataset_test[:, samples_indexs]
    X = dataset[:, samples_indexs]

    # rel_r_m is the label
    labels_indexs = (2)
    labels_header = ['rel_r_m']
    y_train = dataset_train[:,labels_indexs ]
    y_test = dataset_test[:,labels_indexs ]
    y = dataset[:, labels_indexs ]

    # create a dataframe for future use.
    df_X = createDataframe(X_train, t_train, X_test, t_test, samples_header)
    df_y = createDataframe(y_train, t_train, y_test, t_test, labels_header)

    # creating model
    now_time = time.time()
    print('Preprocessing took ' + str(now_time-last_time))
    last_time = now_time
    print ('....................................................................... ')
    print('Creating models for tag ' + bestTag)
    print('samples ' + str(samples))
    print('features ' + str(features))
    print("Remember this is a normalized regressor!!!")

    # scale data to prevent bias
    # Note that scaler is defined acording to the whole dataset!

    scaler = StandardScaler()
    scaler = scaler.fit(X)
    #
    X_train_scaled = scaler.transform(X_train)
    X_test_scaled = scaler.transform(X_test)

    '''
    We are going to model the relationship between distance, phase and frequency
    
    Equation is
     
            ph = (4*pi*f/(c*d)  ) % pi
            
    Hence we have a periodic relationship.
    
    We will use ExpSineSquared and White noise kernel    
    ExpSineSquared explains phase periodicity
    And we add noise to the model

    '''

    alphas = [0.1]     #np.logspace(0, -5, 5)
    alphas_e = [180.0]
    gps = []
    R2s = []
    alpha_es = []
    alpha_ws = []
    for alpha_e in alphas_e:
            for alpha_w in alphas:
                kern = ExpSineSquared(periodicity=alpha_e) + WhiteKernel(noise_level=alpha_w)
                t = time.time()
                gp = GaussianProcessRegressor(n_restarts_optimizer=10,
                                              normalize_y=True, kernel=kern)
                gp.fit(X_train_scaled, y_train)
                R2 = gp.score(X_test_scaled, y_test)
                elapsed = time.time() - t
                print("Train with alphas (E,W) [%.1e, %1.e] lasted %3.3f seconds. R2 is %3.3f" % (
                    alpha_e, alpha_w, elapsed, R2))
                gps.append(gp)
                R2s.append(R2)
                alpha_es.append(alpha_e)
                alpha_ws.append(alpha_w)
                print(". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .")
    selected_model_index = np.argmax(R2s)
    gp = gps[selected_model_index]
    alpha_e = alpha_es[selected_model_index]
    alpha_w = alpha_ws[selected_model_index]
    print('..................................................................................')
    print('Best model has alpha [' + str(alphas[selected_model_index]) + '] R2 ['+str(R2)+']')
    print("Best model has alphas (R,E,W) [%.1e ,%.1e, %1.e] and R2 [%3.3f]" % (
        alpha_e, alpha_w, R2))
    print('..................................................................................')
    saved_data = (gp, df_X, df_y, scaler)
    pickle.dump(saved_data, open(modelURI, "wb"))

    #min_saved_data = (freqs, gmm_list, scaler_list)
    #pickle.dump(min_saved_data, open(minmodelURI, "wb"))