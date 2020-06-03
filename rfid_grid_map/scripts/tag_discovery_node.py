#!/usr/bin/env python


import rospy
import operator
from std_msgs.msg import String
from std_msgs.msg import Float32
from rfid_node.msg import TagReading



# Node class.
class tag_discovery():


    def loadROSParams(self):
        self.detected_tag_topic_name = rospy.get_param(
            '~detected_tag_topic_name', '/lastTag')
        self.tag_discovery_coverage_topic_name = rospy.get_param(
            'tag_discovery_coverage_topic_name', '/tag_coverage')

        self.tag_set_list = rospy.get_param('~tag_set', '')

    def initROS(self):
        # topic publishers
        self.tag_discovery_coverage_pub = rospy.Publisher(self.tag_discovery_coverage_topic_name, Float32, queue_size=1)

        # service clients
        
        # service servers
        
        # topic subscribers and other listeners
        self.detected_tag_sub = rospy.Subscriber(self.detected_tag_topic_name, TagReading, self.detected_tag_callback, queue_size=1)
   
    def initTagIDSet(self):
        self.tagIDSet=set()
        self.tagIDSet.update(self.tag_set_list)
        rospy.logwarn("Node [" + rospy.get_name() + "] Tags under survey: ("+ ', '.join(self.tagIDSet) +")")

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes
        self.initTagIDSet()        
        self.detectedTagIDSet=set()

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] entering spin...")

        rospy.spin()
    
    def detected_tag_callback(self, tag_msg):
        tag_id = tag_msg.ID
        if tag_id in self.tagIDSet:
            self.detectedTagIDSet.add(tag_id)
        else:
            rospy.logwarn("Node [" + rospy.get_name() + "] Detected tag not under survey ("+ tag_id +")")
        
        self.tag_discovery_coverage_pub.publish( float(len(self.detectedTagIDSet)) / float(len(self.tagIDSet))  )


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('tag_discovery_node')#, log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        r_s = tag_discovery()
    except rospy.ROSInterruptException: pass
