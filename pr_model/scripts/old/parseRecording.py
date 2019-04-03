#!/usr/bin/env python

'''
DEPRECATED! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
Use parseRosbagsNoPlay instead!
'''


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

from std_msgs.msg import String

class myParser():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.tf = tf.TransformListener()
        self.saveFile = rospy.get_param('modelDataFile') 
    
        self.robotTFName="/base_link"
        self.tagTFPrefix="rfid/tags/"
        rospy.Subscriber('/rfid/rfid_detect', String, self.tagCallback0, queue_size=10000)
        rospy.Subscriber('/lastTag', TagReading, self.tagCallback1, queue_size=10000)
        rospy.Subscriber('/tf', String, self.tfCallback, queue_size=10)

        
        self.do = True
        
        self.dataEntries=[]
        self.loc = dict()
        self.tfprev = []
        self.tgprev = []

        rospy.on_shutdown(self.lastTask)
        rospy.spin()


    def lastTask(self):
         labels = ['Time','ID','rel_x_m', 'rel_y_m', 'rel_yaw_rad', 'freq_khz', 'rssi_dbm', 'phase_deg']
         df = pd.DataFrame.from_records(self.dataEntries, columns=labels)
         rospy.loginfo("Saving data to csv")
         df.to_csv(self.saveFile, index=False)

    def getTagTFName(self,tid):
        return self.tagTFPrefix+tid


    def tfCallback(self,msg):
        for t in msg.transforms:
            if 'tags' in t.child_frame_id:
                fields = t.child_frame_id.split('/')
                tid = fields[2]
                try:
                    now = rospy.Time()
                    # relative position of the tag respect to the antenna
                    #self.tf.waitForTransform(self.robotTFName,self.getTagTFName(tid),  now, rospy.Duration(0.5))
                    rel_pose, rel_quat = self.tf.lookupTransform(self.robotTFName,self.getTagTFName(tid),  now)
                    rel_x = rel_pose[0]
                    rel_y = rel_pose[1]
                    (rel_rol, rel_pitch, rel_yaw) = tf.transformations.euler_from_quaternion(rel_quat)
                    self.loc[tid] = (rel_x,rel_y,rel_yaw)
                    if tid in self.tfprev:
                        del self.tfprev[ self.tfprev.index(tid) ]
                except tf.Exception:
                    if tid not in self.tfprev:
                        rospy.logerr("Can't get position of tag ("+tid+"). Skipping")
                        self.tfprev.append(tid)
            
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
        try:
            (rel_x,rel_y,rel_yaw) = self.loc[tid]
                  
            if self.do:
                rospy.logdebug("Robot Pose:   "+"{:2.2f}".format(rel_x)+" m., "+"{:2.2f}".format(rel_y)+" m., "+"{:2.2f}".format(rel_yaw*180/3.141592)+" deg.")
                rospy.logdebug("Tag ID: [ "+tid+" ]\n")
                rospy.logdebug("Reading Freq:   "+(freq_khz)+" kHz., "+(rssi_db)+" dBm, "+(phase_deg)+" deg.")
                now2= now.to_sec()
                # beware changes in order or unit here should be consistent with lastTask labels
                entry = (now2,tid,rel_x,rel_y,rel_yaw,freq_khz,rssi_db,phase_deg)
                self.dataEntries.append(entry)
                if (len(self.dataEntries)%10) ==0:
                        rospy.loginfo(str(len(self.dataEntries)) + " Entries recorded")
                if tid in self.tgprev:
                    del self.tgprev[ self.tgprev.index(tid) ]
        except KeyError:
            if tid not in self.tgprev:
                rospy.logerr("Detected tag ("+tid+") with no know location. Skipping")
                self.tgprev.append(tid)

# Main function.
if __name__ == '__main__':
        rospy.init_node('myParser_node')
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=myParser()
        except rospy.ROSInterruptException:
            pass



