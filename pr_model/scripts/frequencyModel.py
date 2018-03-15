
#!/usr/bin/env python

import numpy as np
import numpy.linalg as linalg
import pybayes, pybayes.pdfs, pybayes.filters
import rospy
import tf
from std_msgs.msg import String
from tf.transformations import *


'''
Check  https://github.com/tridge/pyUblox/blob/master/pr_particle.py
for inspiration


- State is relative tag  to antena position: 
xt = [x y a]

- Measurement (observation) is what reader returns us:
yt = [rssi phi]

Reader also returns f, but that's not an observation. I follows a pseudo random sequence.
It's more a state element we already know.
Our idea is that is independant (somehow) between them. 
We model it as several independant filters.

- Conditional probability density function (CPdf) of state in t given state in t-1
p(x_t|x_{t-1}) 

Update model. Well, we assume tags are not moving, but robot is. So relative position changes...
I will use a general gaussian cpdf...

p_xt_xtp = pybayes.pdfs.GaussCPdf 
 
- Conditional probability density function (CPdf) of observation in t given state in t 
p(y_t|x_t)  

This is our observation model. Should be built from gathered data

p_yt_xt = pybayes.pdfs.GaussCPdf 


'''




class myFpFFilter():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self,n, pdStatistics,_mean0,_cov0):
        # for the update model
        mean0 = _mean0
        cov0 = _cov0
        # random variable (state) dimension. Occurs to be the same than condition
        self.len_xt = 3

        # for the sensor model
        self.len_yt = 2
        self.pdData = pdStatistics
        self.x_ranges = np.sort(np.unique(self.pdData['rel_x_m'].copy()))
        self.y_ranges = np.sort(np.unique(self.pdData['rel_y_m'].copy()))
        self.a_ranges = np.sort(np.unique(self.pdData['rel_yaw_rad'].copy()))

        self.init_pdf = self.build_init_pdf(mean0,cov0)
        self.p_xt_xtp = self.build_update_model()
        self.p_yt_xt = self.build_sensor_model()
            
        self.pf = pybayes.filters.ParticleFilter(n, self.init_pdf, self.p_xt_xtp, self.p_yt_xt)

        #self.cont=0
        #self.maxcont=4000
        self.prevRobLoc = np.array([0, 0, 0])
        self.tf = tf.TransformListener()
        self.robotTFName='base_link'
        self.mapTFName = 'map'


    def updateRobotPose(self):
        
        try:
            when = rospy.Time()
            # relative position of the tag respect to the antenna
            self.tf.waitForTransform(self.robotTFName,self.mapTFName, when, rospy.Duration(0.5))
            rob_pose, rob_quat = self.tf.lookupTransform(self.robotTFName,self.mapTFName,  when)
            rob_x = rob_pose[0]
            rob_y = rob_pose[1]
            (rob_rol, rob_pitch, rob_yaw) = tf.transformations.euler_from_quaternion(rob_quat)
            self.robLoc = np.array([rob_x,rob_y,rob_yaw])
            
        except tf.Exception:
                rospy.logerr("Can't get robot position. Skipping")


    def variance(self):
        return self.pf.posterior().variance()   
            
    def mean(self):
        return self.pf.posterior().mean()
        
    def addObservation(self, yt):
        yt = np.array(yt)

        #print("Received observation: " + str(yt))
        #print("Observation Data type: (%s) of (%s)\n\t- Dimmensions (%s)" % (type(yt), type(yt[0]), str(yt.shape)))

        self.pf.bayes(yt)

    def getRanges(self,v0,ranges,llimit=-np.inf,hlimit=np.inf):
        if v0<ranges.min():
           vi         = ranges.min()
           vi_prev = llimit
        if v0>ranges.max():
           vi         = hlimit
           vi_prev = ranges.max()-0.1
        else:
            i = np.argmax(ranges>v0)
            vi = ranges[i]
            if i!=0:
                vi_prev = ranges[i-1]
            else:
                vi_prev = llimit
        return (vi,vi_prev)

    def getModelData(self,xt):
        (x0, y0, a0) = xt
        a0 = ( a0 + np.pi) % (2 * np.pi ) - np.pi

        

        (xi, xi_prev) = self.getRanges( x0, self.x_ranges)
        (yi, yi_prev) = self.getRanges( y0, self.y_ranges)
        (ai, ai_prev) = self.getRanges( a0, self.a_ranges,-np.pi,np.pi)

        subSet = self.pdData[(self.pdData['rel_x_m'] <= xi) & (self.pdData['rel_x_m'] > xi_prev) &
                             (self.pdData['rel_y_m'] <= yi) & (self.pdData['rel_y_m'] > yi_prev) &
                             (self.pdData['rel_yaw_rad'] <= ai) & (self.pdData['rel_yaw_rad'] > ai_prev)].copy()
        if subSet.shape[0]!=1:
            rospy.logerr('Non unique match in model...')
            subSet=subSet.head(1)


        if False:        
            print '\nPoint:'
            print 'p (' + str(x0) +', ' + str(y0) +', ' + str(a0)+')' 
        
        if False:#subSet['count'].iloc[0]==0:
            print '\nmodel has no data for point assigned to interval:'
            print 'pi_prev (' + str(xi_prev) +', ' + str(yi_prev) +', ' + str(ai_prev)+')' 
            print 'pi (' + str(xi) +', ' + str(yi) +', ' + str(ai)+')'  
            
        return subSet

    def modelDataMean(self, xt):
        subSet = self.getModelData(xt)
        rssi = subSet['rssi_dbm_m'].iloc[0]
        phi = subSet['phase_deg_m'].iloc[0]
        yt = [rssi, phi]
        
        if False:
            print '\nHas data:'
            print 'rssi (' + str(rssi)+')'
            print 'phi (' + str(phi) +')\n\n\n'

        return yt

    def modelDataCov(self, xt):
        subSet = self.getModelData(xt)
        c00 = subSet['COV00'].iloc[0]
        c01 = subSet['COV01'].iloc[0]
        c10 = subSet['COV10'].iloc[0]
        c11 = subSet['COV11'].iloc[0]
        COV =  [[c00, c01], [c10, c11]]
        return COV

    def build_init_pdf(self,init_mean, init_cov):
        '''
        This builds the pdf used to generate random states for particles
        '''

        #print(
        #"Init pose mean type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(init_mean), type(init_mean[0]), str(init_mean.shape)))
        #print("Init pose cov type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(init_cov), type(init_cov[0, 0]), str(init_cov.shape)))


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

    def sensor_model_mu_f(self,xt):
        ''' Return mean of Gaussian PDF for measurement yt (reader observation) given xt (tag pose).  
            Mean is at the ideal measurement'''

        yt = self.modelDataMean(xt)
        mu = np.array(yt )

        #if (self.cont % self.maxcont) == 0:
        #    print(
        #    "Prediction mean type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(mu), type(mu[0]), str(mu.shape)))
        #    cont = 0
        return mu

    def sensor_model_R_f(self,xt):
        ''' Return covariance matric of Gaussian PDF for measurement yt (reader observation) given xt (tag pose).  
        '''

        yt_cov_Matrix = self.modelDataCov(xt)
        cov =np.matrix(yt_cov_Matrix)

        #if (self.cont % self.maxcont) == 0:
        #    print(
        #    "Prediction cov type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(cov), type(cov[0, 0]), str(cov.shape)))

        return cov


    def update_model_mu_f(self,xtp):
        '''Return mean of Gaussian PDF for state xt given x(t-1).  
        Needs to be redone! mean at old state'''

        # TODO! xtp is previous tag pose. Current tag pose should be xt = xtp + (xrobot-xrobotp)
        self.updateRobotPose()
        if hasattr(self,'robLoc'):
            incRob = self.robLoc - self.prevRobLoc
            #if True:#abs(incRob[0])+abs(incRob[1])+abs(incRob[2])>0.01:
            #    print 'Tag is at: ' + str(xtp[0])+ ', ' + str(xtp[1])+ ', ' + str(xtp[2])+ ' from robot ' 
            #    print 'robot was at: ' + str(self.prevRobLoc[0])+ ', ' + str(self.prevRobLoc[1])+ ', ' + str(self.prevRobLoc[2])
            #    print 'Now       at: ' + str(self.robLoc[0])+ ', ' + str(self.robLoc[1])+ ', ' + str(self.robLoc[2]) 
            
            self.prevRobLoc = self.robLoc
        else:
            incRob = np.array([0, 0, 0])

        mu = self.turnAndTranslate(xtp,incRob)

        #if (self.cont % self.maxcont) == 0:
        #    print(
        #    "State update mean type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (type(mu), type(mu[0]), str(mu.shape)))
        #    cont = 0
        #cont = cont + 1

        return np.array(mu)

    def turnAndTranslate(self,p0,t):
        # translation is robot increment in position and angle
        (tx,ty,ta)=t
        
        # initial tag position x,y is first two elements  
        #Vectors are: x  y  z   w==1
        p = [p0[0], p0[1], 0, 1.0]

        #(+) moves axis away ...
        inc_x = tx
        inc_y = ty

        #(+) turns clockwise
        inc_a = -ta

        # first turns and then translates
        M0 = compose_matrix( angles=[0, 0, inc_a], translate=[inc_x, inc_y, 0])

        p2 = M0.dot(p)

        return (p2[0],p2[1],p0[2]+ta)
        
        
        
        
        
        

    def update_model_R_f(self,xtp):
        '''Return covariance of Gaussian PDF for state xt given x(t-1).
        How random we consider this?'''

        # TODO! this is a randomly chosen number....
        state_cov = 0.5
        cov =np.diag([state_cov, state_cov, state_cov/10.0])

        #if (self.cont % self.maxcont) == 0:
        #    print("State update cov type: (%s) of (%s)\n\t- Dimmensions (%s)\n" % (
        #    type(cov), type(cov[0, 0]), str(cov.shape)))

        return cov
