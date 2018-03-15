#!/usr/bin/env python

'''
To be used with replay_long_model.launch
Stores into csv the rfid readings from Alfie with groundtruth locations

Could be used as inspiration for other modellings... 

'''


import rospy
import tf
import math
import sys
import pandas as pd
from rfid_node.msg import TagReading

from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String


class poseManager():
    def __init__(self,tid):
        self.tid = tid
        self.pose = PoseStamped()
        
    def poseCallback(self,msg):
        self.pose = msg
    
    def getXYYaw(self):
        rel_x = self.pose.pose.position.x
        rel_y = self.pose.pose.position.y    
        (rel_rol, rel_pitch, rel_yaw) = tf.transformations.euler_from_quaternion([self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w])
        return (rel_x,rel_y,rel_yaw) 

class myParser():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.tf = tf.TransformListener()
        self.saveFile = rospy.get_param('modelDataFile') 
    
        self.robotTFName="/base_link"
        self.tagTFPrefix="rfid/tags/"
        rospy.Subscriber('/lastTag', TagReading, self.tagCallback1, queue_size=10000)
        
        self.tagPoses=dict()
        poseM = poseManager('robot')
        self.tagPoses['robot']=poseM
        rospy.Subscriber('/poses/robot', PoseStamped, poseM.poseCallback, queue_size=10)
        
        #subscribe to tag topics:
        topics = rospy.get_published_topics()
        for nam,typ in topics:
            if '/rel_poses/' in nam:
                tid=nam.split('/')[2]
                poseM = poseManager(tid)
                self.tagPoses[tid]=poseM
                rospy.Subscriber(nam, PoseStamped, poseM.poseCallback, queue_size=10)

        self.do = True
        
        self.dataEntries=[]
        
        rospy.on_shutdown(self.lastTask)
        rospy.spin()

    def lastTask(self):
         labels = ['Time','ID','rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm', 'phase_deg','robot_x_m', 'robot_y_m', 'robot_yaw_rad']
         df = pd.DataFrame.from_records(self.dataEntries, columns=labels)
         rospy.loginfo("Saving data to csv")
         df.to_csv(self.saveFile, index=False)

    def tagCallback1(self,data):
        tid = str(data.ID)
        rssi_db = str(data.rssi)
        phase_deg = str(data.phase)
        freq_khz = str(data.frequency)
        
        self.processTagData(tid, rssi_db, phase_deg, freq_khz)
    
    def processTagData(self,tid, rssi_db, phase_deg, freq_khz):
        if self.do:
            now = rospy.Time.now().to_sec()

            (rel_x,rel_y,rel_yaw) = self.tagPoses[tid].getXYYaw()
            (rob_x,rob_y,rob_yaw) = self.tagPoses['robot'].getXYYaw()
                
            rospy.logdebug("Rel Pose to robot:   "+"{:2.2f}".format(rel_x)+" m., "+"{:2.2f}".format(rel_y)+" m., "+"{:2.2f}".format(rel_yaw*180/3.141592)+" deg.")
            rospy.logdebug("Tag ID: [ "+tid+" ]\n")
            rospy.logdebug("Reading Freq:   "+(freq_khz)+" kHz., "+(rssi_db)+" dBm, "+(phase_deg)+" deg.")
    
            # beware changes in order or unit here should be consistent with lastTask labels
            entry = (now,tid,rel_x,rel_y,rel_yaw,freq_khz,rssi_db,phase_deg,rob_x,rob_y,rob_yaw)
            self.dataEntries.append(entry)
    
            if (len(self.dataEntries)%10) ==0:
                rospy.loginfo(str(len(self.dataEntries)) + " Entries recorded")

# Main function.
if __name__ == '__main__':
        rospy.init_node('myParser_node')
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=myParser()
        except rospy.ROSInterruptException:
            pass



