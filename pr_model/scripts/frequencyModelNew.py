
#!/usr/bin/env python

import numpy as np
import numpy.linalg as linalg
import pybayes, pybayes.pdfs, pybayes.filters
import rospy
import tf
from std_msgs.msg import String
from tf.transformations import *
from sklearn.mixture import GMM

from sklearn.preprocessing import StandardScaler

'''
Check  https://github.com/tridge/pyUblox/blob/master/pr_particle.py
for inspiration


- State is relative tag  to antena position: 
xt = [r t a]

- Measurement (observation) is what reader returns us:
yt = [rssi phi]

- Conditional probability density function (CPdf) of state in t given state in t-1 (Update model)
p(x_t|x_{t-1}) 

Well, we assume tags are not moving. I will use a general gaussian cpdf with some noise...

p_xt_xtp = pybayes.pdfs.GaussCPdf 
 
- Conditional probability density function (CPdf) of observation in t given state in t (observation model)
p(y_t|x_t)  

Should be built from gathered data

p_yt_xt = pybayes.pdfs.GaussCPdf 


'''


class myFpFFilter():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self, n, gmm, scaler,  _mean0, _cov0, freq_khz, txp_dbm):
        # for the update model
        mean0 = _mean0
        cov0 = _cov0
        # random variable (state) dimension. Occurs to be the same than condition
        self.len_xt = 3

        # for the sensor model
        self.len_yt = 2
        self.gmm = gmm
        self.scaler = scaler

        self.init_pdf = self.build_init_pdf(mean0, cov0)
        self.p_xt_xtp = self.build_update_model()
        self.p_yt_xt = self.build_sensor_model()

        self.pf = pybayes.filters.ParticleFilter(n, self.init_pdf, self.p_xt_xtp, self.p_yt_xt)

        # self.cont=0
        # self.maxcont=4000
        self.prevRobLoc = np.array([0, 0, 0])
        self.tf = tf.TransformListener()
        self.robotTFName = 'base_link'
        self.mapTFName = 'map'

        self.txp_dbm = txp_dbm
        self.waveLen = 299792458.0 / (1000.0 * freq_khz)
        self.waveLenFactor = 20.0 * np.log10(self.waveLen)

        self.gainDict = {
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

    def updateRobotPose(self):

        try:
            when = rospy.Time()
            # relative position of the tag respect to the antenna
            self.tf.waitForTransform(self.robotTFName, self.mapTFName, when, rospy.Duration(0.5))
            rob_pose, rob_quat = self.tf.lookupTransform(self.robotTFName, self.mapTFName, when)
            rob_x = rob_pose[0]
            rob_y = rob_pose[1]
            (rob_rol, rob_pitch, rob_yaw) = tf.transformations.euler_from_quaternion(rob_quat)
            self.robLoc = np.array([rob_x, rob_y, rob_yaw])

        except tf.Exception:
            rospy.logerr("Can't get robot position. Skipping")

    def variance(self):
        return self.pf.posterior().variance()

    def mean(self):
        return self.pf.posterior().mean()

    def addObservation(self, yt):
        yt = np.array(yt)
        print("Received observation: " + str(yt))
        print("Observation Data type: (%s) of (%s)\n\t- Dimmensions (%s)" % (type(yt), type(yt[0]), str(yt.shape)))

        self.pf.bayes(yt)


    def dist(self, xt):
        (x0, y0, a0) = xt

        # avoid log(0)
        if x0 == y0 == 0:
            x0 = x0 + 0.0001

        return np.sqrt((x0 * x0) + (y0 * y0))

    def antennasGain(self, xt):
        (x0, y0, a0) = xt

        adeg0 = a0 * 180.0 / np.pi
        adegRound = int(5 * np.round(adeg0 / 5))

        try:
            totalGain = self.gainDict[str(adegRound)] - 30
        except KeyError:
            totalGain = -50

        return totalGain

    # atenuation due to propagation in free space
    def distFactor(self, d):
        return 20.0 * np.log10(4.0 * np.pi * d)

    # if we dont have data, use equations ...
    def friisEq(self, xt, d):
        rxp_dbm = self.txp_dbm + self.antennasGain(xt) + self.waveLenFactor - self.distFactor(d)
        return rxp_dbm

    def propagatedWaveDelay(self, d):
        phi = (4.0 * np.pi * d / self.waveLen) % (2.0 * np.pi)
        return phi

    def modelDataMean(self, xt):
        (x,y,h)  = xt
        r = np.sqrt(x*x + y*y)
        t = np.arctan2(y,x)


        xts = self.scaler.scale([r, t, h, 0, 0])

        xts = self.scaler.inverse_transform(gmm.sample(1))


        self.gmm.

        yt = [rssi, phi]

        if False:
            print '\nHas data:'
            print 'rssi (' + str(rssi) + ')'
            print 'phi (' + str(phi) + ')\n\n\n'

        return yt

    def modelDataCov(self, xt):
        subSet = self.getModelData(xt)

        if subSet['count'].iloc[0] != 0:
            c00 = subSet['COV00'].iloc[0]
            c01 = subSet['COV01'].iloc[0]
            c10 = subSet['COV10'].iloc[0]
            c11 = subSet['COV11'].iloc[0]
        else:
            c00 = 1
            c01 = 0.1
            c10 = 0.1
            c11 = 1

        COV = [[c00, c01], [c10, c11]]
        return COV

    def build_init_pdf(self, init_mean, init_cov):
        '''
        This builds the pdf used to generate random states for particles
        '''

        # print(
        # "Init pose mean type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(init_mean), type(init_mean[0]), str(init_mean.shape)))
        # print("Init pose cov type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(init_cov), type(init_cov[0, 0]), str(init_cov.shape)))

        i_pdf = pybayes.pdfs.GaussPdf(init_mean, init_cov)

        return i_pdf

    def build_update_model(self):
        '''
        Returns Conditional probability of state in t given state in t-1
        We assume tags are not moving, but robot is. So relative position changes...
        I will use a general gaussian cpdf...
        '''
        p_xt_xtp = pybayes.pdfs.GaussCPdf(self.len_xt, self.len_xt, self.update_model_mu_f, self.update_model_R_f)

        return p_xt_xtp

    def build_sensor_model(self):
        '''
        Returns Conditional probability of observation in t given state in t.
        This is our observation model. Should be built from gathered data
        I will use a general gaussian cpdf...
        '''

        p_yt_xt = pybayes.pdfs.GaussCPdf(self.len_yt, self.len_xt, self.sensor_model_mu_f, self.sensor_model_R_f)

        return p_yt_xt

    def sensor_model_mu_f(self, xt):
        ''' Return mean of Gaussian PDF for measurement yt (reader observation) given xt (tag pose).
            Mean is at the ideal measurement'''

        yt = self.modelDataMean(xt)
        mu = np.array(yt)

        # if (self.cont % self.maxcont) == 0:
        #    print(
        #    "Prediction mean type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(mu), type(mu[0]), str(mu.shape)))
        #    cont = 0
        return mu

    def sensor_model_R_f(self, xt):
        ''' Return covariance matric of Gaussian PDF for measurement yt (reader observation) given xt (tag pose).
        '''

        yt_cov_Matrix = self.modelDataCov(xt)
        cov = np.matrix(yt_cov_Matrix)

        # if (self.cont % self.maxcont) == 0:
        #    print(
        #    "Prediction cov type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(cov), type(cov[0, 0]), str(cov.shape)))

        return cov

    def update_model_mu_f(self, xtp):
        '''Return mean of Gaussian PDF for state xt given x(t-1).
        Needs to be redone! mean at old state'''

        # TODO! xtp is previous tag pose. Current tag pose should be xt = xtp + (xrobot-xrobotp)
        self.updateRobotPose()
        if hasattr(self, 'robLoc'):
            incRob = self.robLoc - self.prevRobLoc
            # if True:#abs(incRob[0])+abs(incRob[1])+abs(incRob[2])>0.01:
            #    print 'Tag is at: ' + str(xtp[0])+ ', ' + str(xtp[1])+ ', ' + str(xtp[2])+ ' from robot '
            #    print 'robot was at: ' + str(self.prevRobLoc[0])+ ', ' + str(self.prevRobLoc[1])+ ', ' + str(self.prevRobLoc[2])
            #    print 'Now       at: ' + str(self.robLoc[0])+ ', ' + str(self.robLoc[1])+ ', ' + str(self.robLoc[2])

            self.prevRobLoc = self.robLoc
        else:
            incRob = np.array([0, 0, 0])

        mu = self.turnAndTranslate(xtp, incRob)

        # if (self.cont % self.maxcont) == 0:
        #    print(
        #    "State update mean type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(mu), type(mu[0]), str(mu.shape)))
        #    cont = 0
        # cont = cont + 1

        return np.array(mu)

    def turnAndTranslate(self, p0, t):
        # translation is robot increment in position and angle
        (tx, ty, ta) = t

        # initial tag position x,y is first two elements
        # Vectors are: x  y  z   w==1
        p = [p0[0], p0[1], 0, 1.0]

        # (+) moves axis away ...
        inc_x = tx
        inc_y = ty

        # (+) turns clockwise
        inc_a = -ta

        # first turns and then translates
        M0 = compose_matrix(angles=[0, 0, inc_a], translate=[inc_x, inc_y, 0])

        p2 = M0.dot(p)

        return (p2[0], p2[1], p0[2] + ta)

    def update_model_R_f(self, xtp):
        '''Return covariance of Gaussian PDF for state xt given x(t-1).
        How random we consider this?'''

        # TODO! this is a randomly chosen number....
        state_cov = 0.5
        cov = np.diag([state_cov, state_cov, state_cov / 10.0])

        # if (self.cont % self.maxcont) == 0:
        #    print("State update cov type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (
        #    type(cov), type(cov[0, 0]), str(cov.shape)))

        return cov
