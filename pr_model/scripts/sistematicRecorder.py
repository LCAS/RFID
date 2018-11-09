#!/usr/bin/env python


import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

import tf
import actionlib
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction


# from youbot ....
def create_pose(x, y, yaw):
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(*quat.tolist())
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
  
        return pose

def getSamplingVectors(x0,y0,xN,yN,numPointsX=0,numPointsY=0,numPointsA=0,stepX=0,stepY=0,stepA=0):
    # numPoints mode: xN and yN are included in x y vectors. However 2pi is not
    # step mode: upper limit may not be included ...
    amax = 2.0 * np.pi

    byNumPoints = (numPointsA!= 0) and ( numPointsX != 0) and ( numPointsY != 0)
    byStep = (stepX!= 0) and ( stepY != 0) and ( stepA != 0)

    if byNumPoints and byStep:
        print 'Using byStep by default'

    if byNumPoints:
        x = np.linspace(x0, xN, numPointsX)
        y = np.linspace(y0, yN, numPointsY)
        a = np.linspace(0, amax, numPointsA+1)
        # 2pi == 0
        a = a[0:-1]

    if byStep:
        x = np.arange(x0, xN+stepX, stepX)
        y = np.arange(y0, yN+stepY, stepY)
        a = np.arange(0, amax, stepA)
        # 2pi == 0
        a = a[0:-1]

    nx = len(x)
    ny = len(y)
    na = len(a)
    # print x
    # print y
    # print a

    xv, yv = np.meshgrid(x, y, sparse=False, indexing='ij')
    # This way the robot goes up one cell and goes down y ...
    yv[1::2,:]=yv[1::2,::-1]


    return (xv,yv,x,y,a,nx,ny,na)

def performReading():
    # TODO!!! 
    # get relative tag poses

    # attempt 100 readings

    # store returned values


# poses are relative to robot initial position.

# x,y endpoints are included! but not a


rospy.init_node('robot_move')



client = actionlib.SimpleActionClient('move_base',MoveBaseAction)


#stating point
x0 = 1.0
y0 = 2.0

#end point 
xmax = 3.0
ymax = 4.0

# same resolution in 2D
step = 0.3
stepA= 30.0 * (np.pi/180.0)

(xv,yv,x,y,a,nx,ny,na) = getSamplingVectors(x0,y0,xmax,ymax,stepX=step,stepY=step,stepA=stepA)

while not rospy.is_shutdown():

    client.wait_for_server()

    goal = MoveBaseGoal()

     # iterate through goals        
    for i in range(nx):
        for j in range(ny):
            for ai in a:
                print '\nSample at {0}, {1}, {2}'.format(xv[i,j], yv[i,j],ai * (180.0/np.pi) )
                target =  create_pose(xv[i,j], yv[i,j],ai)

                goal.target_pose = target
                client.send_goal(goal)
                client.wait_for_result(rospy.Duration.from_sec(60.0))

                # Prints out the result of executing the action
                ps = client.get_result()  
                print ps

                if ps.success:
                    performReading()


                rospy.sleep(3)
