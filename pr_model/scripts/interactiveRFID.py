#!/usr/bin/env python


'''
'''

import rospy
from geometry_msgs.msg import PoseStamped
from rfid_node.msg import TagReading


#used for visualization
import colorsys
from visualization_msgs.msg import Marker

# Node example class.


class RFIDDrawerNode():
    
    def grayscale(self,val):
        # val is usually between -100 and -50
        val = int(val)
        val = float(100+val)/60.0
        
        r = g = b = val
        if val>1.0:
            r = 1.0
            g = 0
            b = 0
        if val<0.0:
            r = 0
            g = 0
            b = 1.0
        
        return (r,g,b)#(r/100.0,g/100.0,b/100.0)      
    
    
    def pseudocolor2(self,val):
        # val is usually between -100 and -50
        val = int(val)
        (r,g,b) =  self.pseudocolor(int(100.0*float(100+val)),0,100)
        return (r,g,b)#(r/100.0,g/100.0,b/100.0)  
        
    # https://stackoverflow.com/questions/10901085/range-values-to-pseudocolor
    def pseudocolor(self,val, minval, maxval):
        # convert val in range minval..maxval to the range 0..120 degrees which
        # correspond to the colors red..green in the HSV colorspace
        h = (float(val-minval) / (maxval-minval)) * 120
        # convert hsv color (h,1,1) to its rgb equivalent
        # note: the hsv_to_rgb() function expects h to be in the range 0..1 not 0..360
        r, g, b = colorsys.hsv_to_rgb(h/360, 1., 1.)
        return r, g, b
        

        
        
    '''
    
    '''    
    def create(self):
        markersArray = MarkerArray()
        markersCounter = 0
        
        
        # center marker: sphere and arrow, to distinguish from samples
        centerMarker = Marker()
        centerMarker.header.frame_id = self.lastCentre.header.frame_id
        centerMarker.header.stamp    = rospy.get_rostime()
        centerMarker.ns = "centre"
        centerMarker.id = markersCounter
        markersCounter = markersCounter + 1
        centerMarker.type = centerMarker.SPHERE
        centerMarker.action = centerMarker.ADD
        centerMarker.pose = self.lastCentre.pose
        centerMarker.scale.x = 0.1
        centerMarker.scale.y = 0.1
        centerMarker.scale.z = 0.1
        
        centerMarker.color.r = 0.0
        centerMarker.color.g = 0.5
        centerMarker.color.b = 1.0
        centerMarker.color.a = 1.0

        centerMarker.lifetime = rospy.Duration(0)
        markersArray.markers.append(centerMarker)

        centerMarker = Marker()
        centerMarker.header.frame_id = self.lastCentre.header.frame_id
        centerMarker.header.stamp    = rospy.get_rostime()
        centerMarker.ns = "centre"
        centerMarker.id = markersCounter
        markersCounter = markersCounter + 1
        centerMarker.type = centerMarker.ARROW
        centerMarker.action = centerMarker.ADD
        centerMarker.pose = self.lastCentre.pose
        centerMarker.scale.x = 0.4
        centerMarker.scale.y = 0.04
        centerMarker.scale.z = 0.04
        
        centerMarker.color.r = 0.0
        centerMarker.color.g = 0.5
        centerMarker.color.b = 1.0
        centerMarker.color.a = 1.0

        centerMarker.lifetime = rospy.Duration(0)
        markersArray.markers.append(centerMarker)

        # tuples marker ................................................
        
        for conf_i,pose_i,relPose_i in self.lastTuples:
            #rospy.logdebug("x")
            sample_i = Marker()        
            # common part
            sample_i.lifetime = rospy.Duration(0)
            sample_i.header.stamp    = rospy.get_rostime()
            sample_i.ns = "samples"

            sample_i.type = sample_i.ARROW
            sample_i.action = sample_i.ADD
            sample_i.scale.x = 0.5
            sample_i.scale.y = 0.05
            sample_i.scale.z = 0.05            
            # custom part
            sample_i.id = markersCounter
            markersCounter = markersCounter + 1
            sample_i.header.frame_id = pose_i.header.frame_id
            sample_i.pose = pose_i.pose
            sample_i.pose.position.z = sample_i.pose.position.z -0.03 
            if conf_i == -1:
                (r,g,b) = (0,0,0) 
            else:
                (r,g,b) = self.pseudocolor2(conf_i)
            sample_i.color.r = r
            sample_i.color.g = g
            sample_i.color.b = b
            sample_i.color.a = 1.0
            markersArray.markers.append(sample_i)
        self.markersPub.publish(markersArray) 
   
    # Load Ros parameters
    def loadROSParams(self):
        try:
            # load ros parameters 
            self.markersTopic = 'markers_topic'
        except KeyError as e:
            rospy.logerr("[%s] Can't load rosparam (%s)",rospy.get_name(),str(e))
        
        
    def plotMarker(self,where,rssi_db,tid,freq_khz):
        try:
            centerMarker = Marker()
            centerMarker.header.frame_id = where.header.frame_id
            centerMarker.header.stamp    = rospy.get_rostime()
            centerMarker.ns = "RFID" #"/"+tid+"/"+freq_khz
            centerMarker.id = self.markersCounter
            self.markersCounter = self.markersCounter + 1
            centerMarker.type = centerMarker.ARROW
            centerMarker.action = centerMarker.ADD
            centerMarker.pose = where.pose
            centerMarker.scale.x = 0.3
            centerMarker.scale.y = 0.01
            centerMarker.scale.z = 0.01
            #scale.x is the arrow length, 
            #scale.y is the arrow width 
            #scale.z is the arrow height.
            
            #(r,g,b) = self.pseudocolor2(rssi_db)
            (r,g,b) = self.grayscale(rssi_db)
            centerMarker.color.r = r
            centerMarker.color.g = g
            centerMarker.color.b = b
            centerMarker.color.a = 1.0
                

            centerMarker.lifetime = rospy.Duration(0)
            self.markersPub.publish(centerMarker)
        except:
            pass
                        
    def tagCallback1(self,data):
        self.tid = str(data.ID)
        self.rssi_db = str(data.rssi)
        self.phase_deg = str(data.phase)
        self.freq_khz = str(data.frequency)
        self.plotMarker(self.pose,self.rssi_db,self.tid,self.freq_khz)

    def poseCallback(self,msg):
        self.pose = msg               
        
    # class constructor.
    def __init__(self):
        self.loadROSParams()
        
        self.markersCounter = 0
        
        rospy.Subscriber('/poses/robot', PoseStamped, self.poseCallback, queue_size=10)
        rospy.Subscriber('/lastTag', TagReading, self.tagCallback1, queue_size=10000)

        self.markersPub = rospy.Publisher(self.markersTopic, Marker)
        
        
        # start sampler service callback
       
        rospy.loginfo("Ready...")
        rospy.spin()    

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('lalala_node')# , log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        a2o = RFIDDrawerNode()
    except rospy.ROSInterruptException:
        pass
