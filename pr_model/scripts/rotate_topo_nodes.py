#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import topological_navigation.msg
from geometry_msgs.msg import Twist
import math

class topol_nav_client(object):
    
    def __init__(self) :
        # hardcoded params
        self.rot_speed = 0.25
        self.numWayPoints = 17
        
        
        rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
    
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")
    
        navgoal = topological_navigation.msg.GotoNodeGoal()
    
        points = 0  
        while not rospy.is_shutdown():
            # incremental
            i = points % self.numWayPoints
            rounds = points / self.numWayPoints
            points = rounds + 1

            targ = 'WayPoint'+str(i)
            
            print "Requesting Navigation to %s in round %s" %(targ,str(rounds))
        
            navgoal.target = targ
            #navgoal.origin = orig
        
            # Sends the goal to the action server.
            self.client.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
        
            # Waits for the server to finish performing the action.
            self.client.wait_for_result()
        
            # Prints out the result of executing the action
            ps = self.client.get_result()  # A FibonacciResult
            print ps
            
            #rotate on position
            self.rotateHere()
    
    def rotateHere(self):
            vel_msg = Twist()
    
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.rot_speed
            self.velocity_publisher.publish(vel_msg)

            # wait to turn
            t = 2.0 * math.pi / vel_msg.angular.z
            print 'At %s: rotating for %s secs' % (targ,str(t/10.0))                        
            for i in range(0,30):
                print '.'
                rospy.sleep(t/10.0)
                self.velocity_publisher.publish(vel_msg)

            # and stop
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
           
        
    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('cycle_topos')
    ps = topol_nav_client()
    
