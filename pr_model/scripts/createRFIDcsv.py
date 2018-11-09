#!/usr/bin/env python

'''
Subscribes to rfid readings topic and to a 
list of rfid rel location ground truths
Builds a csv containing:

['Time','ID','rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm', 'phase_deg']

That file can be used for further analysis

'''


import rospy
import tf
import math
import sys
import pandas as pd
from rfid_node.msg import TagReading
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String


class TagTracker():

    def __init__(self,tagid):
        self.id = tagid
        self.hasPose = False

    def posesCallback(self,msg):
        self.rel_pose = msg.pose
        self.hasPose = True
        self.rel_x   = msg.pose.position.x
        self.rel_y   = msg.pose.position.y

        rel_quat = [ msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z, msg.pose.orientation.w  ]
        (rel_rol, rel_pitch, rel_yaw) = tf.transformations.euler_from_quaternion(rel_quat)

        self.rel_yaw = rel_yaw

class myParser():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.tf = tf.TransformListener()
        self.saveFile = rospy.get_param('csvDataFile','output.csv')     
        self.tagTrackersDict = dict()
        self.dataEntries=[]

        self.subscribeToRelPoses('/rel_poses')
        rospy.Subscriber('/rfid/rfid_detect', String, self.tagCallback0, queue_size=10000)
        rospy.Subscriber('/lastTag', TagReading, self.tagCallback1, queue_size=10000)
        
        rospy.on_shutdown(self.lastTask)

        self.do = True
        rospy.spin()

    def subscribeToRelPoses(self,topicPrefix):
        topicPrefix = '/rel_poses'
        allTopics = rospy.get_published_topics()
        for entry in allTopics:
                topic = entry[0]
                ttype = entry[1]
                if topicPrefix in entry[0]:
                        tag_id = topic.split('/')[2]
                        rospy.loginfo("Creating tracker for tag: "+tag_id)               
                        tt = TagTracker(tag_id)
                        rospy.Subscriber(topic, PoseStamped, tt.posesCallback, queue_size=10000)
                        self.tagTrackersDict[tag_id] = tt


    def retrieveXYY(self,tid):
        try:
               tt = self.tagTrackersDict[tid]
               if tt.hasPose:
                  (rel_x,rel_y,rel_yaw) = (tt.rel_x,tt.rel_y,tt.rel_yaw)
                  return (rel_x,rel_y,rel_yaw)
               else:
                  rospy.logerr_throttle(10,"Havent received pose yet for tag "+tid)
        except KeyError:
               rospy.logerr_throttle(10,"Unknown tag pose "+tid)




    def lastTask(self):
         labels = ['Time','ID','rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm', 'phase_deg']
         df = pd.DataFrame.from_records(self.dataEntries, columns=labels)
         rospy.loginfo("Saving data to csv")
         df.to_csv(self.saveFile, index=False)

            
    def tagCallback1(self,data):
        tid = str(data.ID)
        rssi_db = str(data.rssi)
        phase_deg = str(data.phase)
        freq_khz = str(data.frequency)
        
        self.processTagData(tid, rssi_db, phase_deg, freq_khz)
    
    def tagCallback0(self, data):

        rawD = data.data
        fields = rawD.split(':')
        
        tid = fields[1]
        rssi_db = fields[2]
        phase_deg = fields[3]
        freq_khz = fields[4]
        
        self.processTagData(tid, rssi_db, phase_deg, freq_khz)
    
    def processTagData(self,tid, rssi_db, phase_deg, freq_khz):
        
        now = rospy.Time.now()

        xyw = self.retrieveXYY(tid)
                  
        if self.do and (xyw is not None):
                (rel_x,rel_y,rel_yaw) = xyw
                rospy.logdebug("Robot Pose:   "+"{:2.2f}".format(rel_x)+" m., "+"{:2.2f}".format(rel_y)+" m., "+"{:2.2f}".format(rel_yaw*180/3.141592)+" deg.")
                rospy.logdebug("Tag ID: [ "+tid+" ]\n")
                rospy.logdebug("Reading Freq:   "+(freq_khz)+" kHz., "+(rssi_db)+" dBm, "+(phase_deg)+" deg.")
                now2= now.to_sec()
                # beware changes in order or unit here should be consistent with lastTask labels
                entry = (now2,tid,rel_x,rel_y,rel_yaw,freq_khz,rssi_db,phase_deg)
                self.dataEntries.append(entry)

                #periodic status update
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



