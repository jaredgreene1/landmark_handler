#!/usr/bin/env python

import actionlib
import rospy
import yaml
import sys
import atexit
import time
from datetime import datetime

from math import sin, cos, fabs

from nav_msgs.msg import Odometry
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from spoken_interaction.msg import VerbalRequest, VerbalResponse, KeyValue
from visualization_msgs.msg import Marker

areaThreshold = 5

def storeLoc(globLoc):
    global currentGlobalPose
    currentGlobalPose = globLoc.pose.pose

def navToLandmark(lm_known, lm):
    move_base.goto(lm_known[lm], 0)

# If further than thresh then it is a different landmark
def diff_landmarks(old_pos, new_pos):
    if (fabs(old_pos.x - new_pos.x) > areaThreshold) or \
        (fabs(old_pos.y - new_pos.y) > areaThreshold) or \
        (fabs(old_pos.z - new_pos.z) > areaThreshold):
            return True
    else:
        return False

# If key exists, are they diff? If so, overwrite?
def alreadyKnown(lm_known, lm_new, current_pos):
    if not lm_new in lm_known:
        return False
    else:
        if diff_landmarks(current_pos, lm_known[lm_new]):
            return True 
        else:
            return False 


def constructLandmark(lm_known, lm_new,  globalPose, lm_pub):
    pos = globalPose.position
    
    if alreadyKnown(lm_known, lm_new, pos):
        return ("I already know a place with that name on this map." + \
        "Should I redefine it?", "landmarkOverwrite")
    else:
        # Add to known  landmarks 
        lm_known[lm_new] = pos
        
        # make and publish the marker
        mrk = Marker()
        mrk.ns = "MAP NAME?"
        mrk.id = int(time.time()) 
        mrk.type = 9
        mrk.action = 0
        mrk.pose = globalPose
        mrk.scale.x = 1 
        mrk.scale.y = 1
        mrk.scale.z = 1
        mrk.color.r = .5
        mrk.color.g = .5
        mrk.color.b = .5
        mrk.color.a = .2
        mrk.lifetime= rospy.Duration(0)
        mrk.frame_locked = True
        mrk.text = lm_new
        lm_pub.publish(mrk)
        return("I'll remember that this is "  + lm_new + ". Thanks!", '')

def verbalReqHandler(request):
    print "I heard you say: %s" % request.phrase
    resp            = VerbalResponse()
    resp.timestamp  = str(datetime.now())
    resp.request_id = request.timestamp
    response        = "No landmark action recognized"    
    context         = ''

    #Check if I care about the new verbal command
    if request.action_id == "navigate_to_landmark":
        lm = None
        for p in request.params:
            if p.key == 'landmark':
                lm = p.value
        if lm in lm_known:
            response, context = navToLandmark(lm_known, lm)
        else:
            response = "I dont't know where that is!"
    
    if request.action_id == "create_landmark":
        for p in request.params:
            if p.key == 'landmark':
                lm = p.value
                response, context = constructLandmark(lm_known, lm, currentGlobalPose, landmarkMarker)
    
    
    if request.action_id == "coord_nav":
        pos = Point()
        pos.x = int(request.params[0].value)
        pos.y = int(request.params[1].value)
        pos.z = 0
        move_base.goto(pos, 0)
    resp.verbal_response = response
    resp.context         = context
    vocalResponse.publish(resp)


class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, lm_pos, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = lm_pos.x 
        move_goal.target_pose.pose.position.y = lm_pos.y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()



def handle_exit():
    f = open(sys.argv[1], 'w+')
    yaml.dump(lm_known, f)
    f.close()

if __name__ == "__main__":
    
    # Create a node
    rospy.init_node("demo")    
    currentGlobalPose = None

    # Load known landmarks for the map
    if sys.argv == 1:
        rospy.loginfo("no landmark path provided")
        exit(0)
    landmarksFile = open(sys.argv[1], 'r')
    lm_known = yaml.load(landmarksFile)
    if not lm_known:
        lm_known = {}
    landmarksFile.close()
    
    try:
        # Setup clients, subscribers, publishers
        move_base = MoveBaseClient()
        globLoc = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, storeLoc)
        landmarkMarker = rospy.Publisher("landmarks", Marker, queue_size=10)
        vocalComman = rospy.Subscriber("verbal_input", VerbalRequest, verbalReqHandler)
        vocalResponse = rospy.Publisher("verbal_response", VerbalResponse, queue_size = 10)

        # Make sure sim time is working
        while not rospy.Time.now():
            pass
        

        rospy.spin()

    # Make sure we save any new landmarks we learned about today
    finally:
        handle_exit()
