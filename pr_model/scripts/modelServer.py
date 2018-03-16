#!/usr/bin/env python

'''
rosservice call /model_server '{state: [1.4,0.0,912250.0], observ: [-60.0,100.0]}'
'''

import rospy
import math
import pandas as pd
import numpy as np


from pr_model.srv import get_prob, get_probRequest, get_probResponse


# Node class.

class modelServerNode():

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

    def getModelData(self,s):
        (x0, y0,f0) = s
        delta = 0.5
        #(xi, xi_prev) = self.getRanges( x0, self.x_ranges)
        #(yi, yi_prev) = self.getRanges( y0, self.y_ranges)
        (xi, xi_prev) = (x0+delta,x0-delta)
        (yi, yi_prev) = (y0+delta,y0-delta)

        subSet = self.pdData[(self.pdData['rel_x_m']  <= xi) & (self.pdData['rel_x_m'] > xi_prev) &
                             (self.pdData['rel_y_m']  <= yi) & (self.pdData['rel_y_m'] > yi_prev) &
                             (self.pdData['freq_khz'] == f0) ].copy()
        
        if False:#subSet.shape[0]==0:
            rospy.logerr('Non unique match in model...')
            subSet=subSet.head(1)

        if False:        
            print '\nPoint:'
            print 'p (' + str(x0) +', ' + str(y0) +', ' + str(f0)+')' 
        
        if False:#len(subSet)==0:
            print '\nmodel has no data for point assigned to interval:'
            print 'pi_prev (' + str(xi_prev) +', ' + str(yi_prev) +', ' + str(f0)+')' 
            print 'pi (' + str(xi) +', ' + str(yi) +', ' + str(f0)+')'  
            
        return subSet

    def getModelDataStats(self, s):
        subSet = self.getModelData(s)

        count = subSet.size
        
        rssi_m0 = -99
        phase_m0 = 0
        COV0 = np.diag([10, 10])


        rssi_m = rssi_m0
        phase_m = phase_m0
        COV = COV0

        #enough data to have averages
        if (count != 0) & (subSet['rssi_dbm'].size > 0):
            count = subSet['rssi_dbm'].size
            rssi_i = subSet['rssi_dbm']
            phase_i = subSet['phase_deg']
            rssi_m = rssi_i.mean()
            phase_m = phase_i.mean()


        # enough data to have covariances
        if (count != 0) & (subSet['rssi_dbm'].size > 1):
            count = subSet['rssi_dbm'].size
            # compute covariance matrix and determinant
            X = np.stack((rssi_i, phase_i), axis=0)
            COV = np.cov(X)
            detC = np.linalg.det(COV)
            # we don't want singular, indefinde matrixes
            if (detC>0):
                # we don't want  definide negative covariance matrixes ...
                try:
                    invC = np.linalg.inv(COV)
                except:
                    COV = COV0
                    rospy.logerr("Singular covariance")
            else:
                COV = COV0
                rospy.logerr("Negative covariance")




        return ([rssi_m,phase_m],COV)


    def getProb(self,state,obs):
        # I read from my model mean m and cov c for state s
        (mean,cov) = self.getModelDataStats(state)


        m = np.array(mean)
        o = np.array(obs)
        
        C =np.matrix(cov)

        # compute logarithm of normalization constant (can be cached somehow in future)
        n =  C.shape[0]
        n2Pi = np.power( (2.0*np.pi),n)        
        detC = np.linalg.det(C)
        den = np.power(n2Pi * detC, 0.5 )

        dif = m - o
        num = np.dot(np.linalg.inv(C), dif)
        num =  -0.5 * np.dot( num,dif)
        num = num[0,0]
        num = np.exp(num)
        return num/den

    def getLikehood(self,get_probReq):
        ans = get_probResponse()
        
        # I receive xrel,yrel,db,ph. s=[xrel,yrel,f]  o=[db,ph]  
        s = get_probReq.state
        o = get_probReq.observ
        
        # pr should be E[o-m]
        ans.prob  = self.getProb(s,o)

        # try:        
        #     lpr =  math.log(pr) 
        # except ValueError:
        #     lpr = -400
        # try:
        #     nlpr =  math.log(1 - pr) 
        # except ValueError:
        #     nlpr = -400

        # ans.llh = (lpr - nlpr)

        return ans

    def printRanges(self):
        rospy.loginfo("X ranges: ({0},{1})".format(self.x_ranges[0],self.x_ranges[-1]) )
        rospy.loginfo("Y ranges: ({0},{1})".format(self.y_ranges[0],self.y_ranges[-1]) )
        rospy.loginfo("a ranges: ({0},{1})".format(self.a_ranges[0]*180.0/np.pi,self.a_ranges[-1]*180.0/np.pi) )
        rospy.loginfo("f ranges: ({0},{1})".format(self.f_ranges[0],self.f_ranges[-1]) )

    def rosSetup(self):

        try:
            # load ros parameters
            self.modelURI = str(rospy.get_param(rospy.get_name() + '/modelURI','/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20db-linda-fab-lab.csv')  )
        except KeyError as e:
            rospy.logerr("[%s] Can't load rosparam (%s)",rospy.get_name(),str(e))

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        rospy.loginfo("Advertising RFID reading likehood service ")

        # get parameters from ros
        self.rosSetup()

        # init stuff
        self.pdData = pd.read_csv(self.modelURI)
        self.x_ranges = np.sort(np.unique(self.pdData['rel_x_m'].copy()))
        self.y_ranges = np.sort(np.unique(self.pdData['rel_y_m'].copy()))
        self.a_ranges = np.sort(np.unique(self.pdData['rel_yaw_rad'].copy()))
        self.f_ranges = np.sort(np.unique(self.pdData['freq_khz'].copy()))

        self.printRanges()


                            
        # start service callbacks
        self.s=rospy.Service('model_server', get_prob, self.getLikehood)

        rospy.loginfo("Ready...")
        rospy.spin()


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('model_server_node')#, log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        gs_s = modelServerNode()
    except rospy.ROSInterruptException: pass
