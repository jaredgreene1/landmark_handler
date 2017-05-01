#!/usr/bin/env python

import actionlib
import rospy
import tf2_ros

from math import sin, cos

from nav_msgs.msg import Odometry
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes

# Move base using navigation stack
def storeLoc(globLoc):
    global x, y 
    x = globLoc.pose.pose.position.x
    y = globLoc.pose.pose.position.y

class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")    
    x, y = 0, 0
    landmarks = {}
    globLoc = rospy.Subscriber("odom", Odometry, storeLoc)
    tf_listen = tf2_ros.TransformListener(tfBuf)

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
  
    try:
        odom2map = tfBuf.lookup_transform('odom','map', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("EXCEPTION ENCOUNTERED IN TRANSFORMATION LOOKUP")

   
    while True:
        action = raw_input("What do you want? [nav or rem]")
        if action == "nav":
            lm = raw_input("Where should I go? [lm name]")
            if lm in landmarks:
                print ("Heading to %s at %d, %d") %(lm, landmarks[lm][0], landmarks[lm][1])
                move_base.goto(landmarks[lm][0], landmarks[lm][1], 1.0)
            else:
                print("I dont't know where that is!")
        if action == "navc":
            xc = raw_input("x coord:")
            yc = raw_input("y coord:")
            tc = raw_input("theta coord:")
            move_base.goto(long(xc), long(yc), long(tc))
        if action == "rem":
            lm = raw_input("Where am I??")
            landmarks[lm] = (x,y)
            print ("I'll remember that %s is at %d and %d") %(lm, int(x), int(y))
            print("here are my landmarks!")
            print landmarks
        
   
   
