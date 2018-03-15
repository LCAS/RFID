#!/usr/bin/env python

'''
Reads rfid readings and uses pf to guess locations
'''


import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from rfid_node.msg import TagReading

import pandas as pd
import numpy as np
from scipy import stats

import frequencyModel

class loca():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # ................................................................
        # Constants

        # model file
        #modelURI = '/home/manolofc/catkin_ws/src/RFID/src/clients/ros/pr_model/tests/long/long_1tag_model.csv'
        #modelURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/30db/30db_model.csv'
        modelURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/linda/20db-linda-fab-lab_model.csv'
       
        # followed tag
        #self.tagID = '300833B2DDD9014000000014'
        self.tagID = '390100010000000000000001'
        # num of particles
        n = 5

        self.pubRate = 2 #seconds
        # TODO initial state guess for tag
        x0 = 2
        y0 = 0
        a0 = -90.0*(np.pi/180.0)
        x0_cov = 0.5
        y0_cov = 0.5
        a0_cov = 0.5

        # ................................................................


        # load models
        models = pd.read_csv(modelURI)

        # frequency info
        self.freqVector = np.sort(np.unique(models['freq_khz'].copy()))
        
        #rospy.loginfo((self.freqVector.tolist()))
        
        
        mean0 = np.array([x0, y0, a0])
        cov0 = np.diag([x0_cov, y0_cov, a0_cov])

        # create one particle filter per frequency
        self.pfilt = []
        
    
        # USE ONLY ONE
        self.freqVector =np.array([self.freqVector[0]])

        for fi in self.freqVector:
            statistics_f = models[models['freq_khz'] == fi].copy()
            pfilti = frequencyModel.myFpFFilter(n, statistics_f, mean0, cov0)
            self.pfilt.append(pfilti)

        rospy.Subscriber('/rfid/rfid_detect', String, self.tagCallback, queue_size=10000)
        rospy.Subscriber('/lastTag', TagReading, self.tagCallbackR, queue_size=10000)
        #publisher for guessed position
        self.br = tf.TransformBroadcaster()
        self.posePub = rospy.Publisher('/tagLoc', PoseStamped, queue_size=10)
        self.poseArrayPub = rospy.Publisher('/particleCloud', PoseArray, queue_size=10)
                  
        #Timer to periodically publish them
        rospy.Timer(rospy.Duration(self.pubRate), self.pose_timerCallback)
        rospy.Timer(rospy.Duration(self.pubRate), self.tf_timerCallback)
        rospy.Timer(rospy.Duration(self.pubRate), self.particle_timerCallback)
        
        
        rospy.spin()


    # publish particles status
    def particle_timerCallback(self, data):        
        # publish it as poseArray
        msg = PoseArray()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = rospy.Time.now()
        for pfi in self.pfilt:
             for i in range(pfi.pf.emp.particles.shape[0]):
                # get ith particle           
                (xf,yf,af) = pfi.pf.emp.particles[i]
                ans = Pose()
                ans.position.x = xf
                ans.position.y = yf
                ans.position.z = 0
                (ans.orientation.x, ans.orientation.y, ans.orientation.z, ans.orientation.w) = tf.transformations.quaternion_from_euler(0, 0, af)
                msg.poses.append(ans)

        self.poseArrayPub.publish(msg) 
            
    # publish most likely pose
    def pose_timerCallback(self, data):
        # read average x,y,a from all filters
        x = []
        y = []
        a = []

        # todo what we really have is a mixture model... combination should be done in a different way ...
        for pfi in self.pfilt:
            (xf,yf,af) = pfi.mean()
            #pfi.variance())

            x.append(xf)
            y.append(yf)
            a.append(af)
        # make the average
        mean_x = np.array(x).mean()
        mean_y = np.array(y).mean()
        mean_a = np.array(a).mean()

        
        # publish it as posestamped
        ans = PoseStamped()
        ans.header.frame_id = 'base_link'
        ans.pose.position.x = mean_x
        ans.pose.position.y = mean_y
        ans.pose.position.z = 0

        (ans.pose.orientation.x, ans.pose.orientation.y, ans.pose.orientation.z, ans.pose.orientation.w) = tf.transformations.quaternion_from_euler(0, 0, mean_a)
        self.posePub.publish(ans)
    
   # TODO we could use covariance as well ... and mixture model

    # publish data as tf
    def tf_timerCallback(self, data):
        # read average x,y,a from all filters
        x = []
        y = []
        a = []

        # todo what we really have is a mixture model... combination should be done in a different way ...
        for pfi in self.pfilt:
            (xf,yf,af) = pfi.mean()
            #pfi.variance())

            x.append(xf)
            y.append(yf)
            a.append(af)
        # make the average
        mean_x = np.array(x).mean()
        mean_y = np.array(y).mean()
        mean_a = np.array(a).mean()
        
        # publish it as tf
        
        self.br.sendTransform((mean_x, mean_y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, mean_a),
                     rospy.Time.now(),
                     self.tagID,'base_link')
    

    def tagCallback(self, data):

        rawD = data.data
        fields = rawD.split(':')
        tid = fields[1]
        rssi_db = fields[2]
        phase_deg = fields[3]
        freq_khz = fields[4]

        if self.tagID in tid:
            # get data
            (yt,f) = ( [int(rssi_db),int(phase_deg) ] , int(freq_khz))
            f_index = self.getIndex(self.freqVector.tolist(),f)

            # perform bayes rule on new observation on corresponding filter
            self.pfilt[f_index].addObservation(yt)
    
    def tagCallbackR(self, data):


        if self.tagID in data.ID:
            
            # get data
            (yt,f) = ( [int(data.rssi),int(data.phase) ] , int(data.frequency))
            f_index = self.getIndex(self.freqVector.tolist(),f)
            
            # USE ONLY ONE
            if f_index == 0: 
                print  'received: '+str(data.rssi)+' db, '+str(data.phase)+' deg, '+str(data.frequency)+' khz '     
                
                # perform bayes rule on new observation on corresponding filter
                try:
                    self.pfilt[f_index].addObservation(yt)
                except AttributeError:
                    pass
                
    def getIndex(self,fv,fi):
        for i in range(0,len(fv)):
            if fv[i] == fi:
                return i
        return -1

# Main function.
if __name__ == '__main__':
        rospy.init_node('locator_node')
        print 'USING ONLY 1 FREQ!'
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=loca()
        except rospy.ROSInterruptException:
            pass



