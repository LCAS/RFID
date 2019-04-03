import time
import numpy as np
import pandas as pd
import statsmodels


fileURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20dB-Linda-FAB-LAB-V3.csv'

print 'dataFile: ' + fileURI

# load data from csv
# colnames: Time,ID,rel_x_m,rel_y_m,rel_yaw_rad,freq_khz,rssi_dbm,phase_deg,robot_x_m,robot_y_m,robot_yaw_rad
allData0 = pd.read_csv(fileURI)

# divide dataset per
tags = np.unique(allData0['ID'])

# all tags are the same, but let's build a model specific to one tag
bestTag = allData0.groupby('ID').size().sort_values().index[-1]
allData = allData0[allData0['ID'] == bestTag]

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
rel_r_m = np.sqrt(rel_x_m.values * rel_x_m.values + rel_y_m.values * rel_y_m.values)
rel_tetha_rad = np.arctan2(rel_y_m.values, rel_x_m.values)

dataset = np.stack((timestamps, freq_khz, rel_r_m, rel_tetha_rad,
                    rel_yaw_rad, rssi_dbm, phase_deg), axis=1)


# freq_khz, rel_r_m, rel_tetha_rad, rel_yaw_rad are samples
X = np.stack((freq_khz, rel_r_m, rel_tetha_rad, rel_yaw_rad), axis=1)

# rssi_dbm, phase_deg  are the label
y = np.stack((rssi_dbm, phase_deg), axis=1)

m = statsmodels.multivariate.manova.MANOVA(y,X)
print m
