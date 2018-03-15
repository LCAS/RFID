
from time import time
import numpy as np

from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn.datasets import load_digits
from sklearn.decomposition import PCA
from sklearn.preprocessing import scale

import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize
import matplotlib.colorbar
import pandas as pd

def uberPlotModel(origURI,offset):
    data = np.loadtxt(open(origURI, "rb"), delimiter=",", skiprows=1)
    #readings = pd.read_csv(origURI)

    # dataMatrix [entry, field]
    # dataMatrix[:,field]

    n_samples, n_features = data.shape

    Count = data[:,10]
    X = data[:,2-offset]
    X = X[Count>0]
    Y = data[:,3-offset]
    Y = Y[Count>0]
    ang = data[:,4-offset]
    ang = ang[Count>0]
    U = np.cos(ang)
    V = np.sin(ang)

    # db
    C = data[:,6-offset]
    C = C[Count>0]
    # phase
    #C = data[:,7-offset]

    norm = Normalize()
    norm.autoscale(C)
    # we need to normalize our colors array to match it colormap domain
    # which is [0, 1]

    colormap = cm.inferno

    fig, ax = plt.subplots()

    ax.quiver(X, Y, U, V,  color=colormap(norm(C)),   scale_units='inches', scale=10)

    cax = fig.add_axes([0.125, 0.925, 0.775, 0.0725])
    matplotlib.colorbar.ColorbarBase(ax=cax, cmap=cm.inferno,
                                 orientation="horizontal")
def uberPlot(origURI,offset):
    data = np.loadtxt(open(origURI, "rb"), delimiter=",", skiprows=1)
    #readings = pd.read_csv(origURI)

    # dataMatrix [entry, field]
    # dataMatrix[:,field]

    n_samples, n_features = data.shape


    X = data[:,2-offset]
    Y = data[:,3-offset]
    ang = data[:,4-offset]
    U = np.cos(ang)
    V = np.sin(ang)

    # db
    C = data[:,6-offset]
    # phase
    #C = data[:,7-offset]

    norm = Normalize()
    norm.autoscale(C)
    # we need to normalize our colors array to match it colormap domain
    # which is [0, 1]

    colormap = cm.inferno

    fig, ax = plt.subplots()

    ax.quiver(X, Y, U, V,  color=colormap(norm(C)),   scale_units='inches', scale=10)

    cax = fig.add_axes([0.125, 0.925, 0.775, 0.0725])
    matplotlib.colorbar.ColorbarBase(ax=cax, cmap=cm.inferno,
                                 orientation="horizontal")
                                 
                                 
# features are ['time', 'ID', 'rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm_m', 'phase_deg_m','COV00','COV01','COV10','COV11','count']


origURI =  '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20db-linda-fab-lab-v2.csv'
uberPlot(origURI,0)

origURI =  '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20db-linda-fab-lab_model.csv'
uberPlotModel(origURI,2)
                                                                  
plt.show()
