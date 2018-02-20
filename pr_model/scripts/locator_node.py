#!/usr/bin/env python

'''
Reads rfid readings and uses pf to guess locations
'''


import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import pandas as pd
import numpy as np

import frequencyModel

class loca():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # ................................................................
        # Constants

        # model file
        #modelURI = '/home/manolofc/catkin_ws/src/RFID/src/clients/ros/pr_model/tests/long/long_1tag_model.csv'
        modelURI = '/home/manolofc/catkin_ws/src/RFID/pr_model/tests/30db/30db_model.csv'

        # followed tag
        self.tagID = '300833B2DDD9014000000014'

        # num of particles
        n = 1000

        self.pubRate = 1 #second
        # TODO initial state guess for tag
        x0 = 0
        y0 = 0
        a0 = 0
        x0_cov = 10
        y0_cov = 10
        a0_cov = 1

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
        for fi in self.freqVector:
            statistics_f = models[models['freq_khz'] == fi].copy()
            pfilti = frequencyModel.myFpFFilter(n, statistics_f, mean0, cov0)
            self.pfilt.append(pfilti)

        rospy.Subscriber('/rfid/rfid_detect', String, self.tagCallback, queue_size=10000)

        #publisher for guessed position
        self.pub = rospy.Publisher('/tagLoc', PoseStamped, queue_size=10)

        #Timer to periodically publish them
        rospy.Timer(rospy.Duration(self.pubRate), self.timerCallback)

        rospy.spin()

    def timerCallback(self, data):
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
        self.pub.publish(ans)

        # TODO we could use covariance as well ... and mixture model

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

    def getIndex(self,fv,fi):
        for i in range(0,len(fv)):
            if fv[i] == fi:
                return i
        return -1

# Main function.
if __name__ == '__main__':
        rospy.init_node('locator_node')
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=loca()
        except rospy.ROSInterruptException:
            pass



