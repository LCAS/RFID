#!/usr/bin/env python

'''
Publishes tag position in robot coordinates.

myFvim

'''


import rospy
import tf
from tf.msg import tfMessage
import math
from geometry_msgs.msg import PoseStamped

class roboPosition():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
   def __init__(self):
        self.loadROSParams()
        self.initROS()
        rospy.spin()

   def loadROSParams(self):
        self.robotTFName =rospy.get_param('~robot_tf')
        self.tagTFName= rospy.get_param('~tag_tf')
        self.relPoseTopic = rospy.get_param('~rel_pose_topic') 
        
   def initROS(self):
        rospy.Subscriber('/tf', tfMessage, self.tfCallback, queue_size=10)
        self.pub = rospy.Publisher(self.relPoseTopic, PoseStamped, queue_size=10)
        self.tf = tf.TransformListener()

   def tfCallback(self,msg):
        for t in msg.transforms:
            if self.tagTFName in t.child_frame_id:
                try:
                    anyTime = rospy.Time()
                    # relative position of the tag respect to the antenna
                    rel_pose, rel_quat = self.tf.lookupTransform(self.robotTFName,self.tagTFName,  anyTime)
                    self.publishRelPose(rel_pose,rel_quat)
                except tf.Exception:
                    pass

   def publishRelPose(self,pose_vec,quat_vec):
       msg = PoseStamped()
       #msg.header.frame_id = self.tagTFName
       msg.header.frame_id = self.robotTFName

       msg.pose.position.x = pose_vec[0]
       msg.pose.position.y = pose_vec[1]
       msg.pose.position.z = pose_vec[2]

       msg.pose.orientation.x = quat_vec[0]
       msg.pose.orientation.y = quat_vec[1]
       msg.pose.orientation.z = quat_vec[2]
       msg.pose.orientation.w = quat_vec[3]

       self.pub.publish(msg)

# Main function.
if __name__ == '__main__':
        rospy.init_node('roboPosition_node')
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=roboPosition()
        except rospy.ROSInterruptException:
            pass



