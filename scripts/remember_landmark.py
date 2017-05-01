#!/usr/bin/env python

import actionlib
import rospy
import yaml
import sys
import atexit
import time
import tf2_ros
import tf
import ast
from datetime import datetime
import random
from math import sin, cos, fabs

from nav_msgs.msg import Odometry
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped, Point
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
    return "Navigated to landmark", ''

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
    tfPub = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = lm_new
    t.transform.translation.x = pos.y
    t.transform.translation.y = pos.x
    t.transform.translation.z = 0
    t.transform.rotation.x    = 0.0
    t.transform.rotation.y      = 0.0
    t.transform.rotation.z      = 0.0 
    t.transform.rotation.w      = 1 
    tfPub.sendTransform(t)

    if alreadyKnown(lm_known, lm_new, pos):
        return ("I already know a place with that name on this map." + \
        "Should I redefine it?", "landmarkOverwrite")
    else:
        # Add to known  landmarks 
        lm_known[lm_new] = pos
        
        # make and publish the marker
        mrk = Marker()
        mrk.header.frame_id = lm_new
        mrk.header.stamp = rospy.Time.now()
        mrk.ns = "/landmarkers"
        mrk.id = int(random.random()*1000)
        mrk.type = 9
        mrk.action = 0
        mrk.pose = globalPose
        mrk.pose.orientation.w = 1
        mrk.scale.x = 5 
        mrk.scale.y = 0 
        mrk.scale.z = 0 
        mrk.color.r = 0.2
        mrk.color.g = 0.4
        mrk.color.b = 1.0
        mrk.color.a = 0.7 
        mrk.lifetime= rospy.Duration(0)
        mrk.frame_locked = True
        mrk.text = lm_new
        lm_pub.publish(mrk)
        return("I'll remember that this is "  + lm_new + ". Thanks!", '')

def verbalReqHandler(request):
    print "I heard you say: %s" % request.phrase
    parameters = request.parameters

    # Set up the responses for request receipt and task completion
    receiptResponse                 = VerbalResponse()
    completionResponse              = VerbalResponse()
    completionResponse.request_id   = receiptResponse.request_id      = request.timestamp
    completionResponse.clientInfo   = receiptResponse.clientInfo      = request.clientInfo
    

    # Will be the results of the actions and returns on task completion
    response            = "No landmark action recognized"    
    context             = ''

    # Nav to Landmark
    if request.action_id == "navigate_to_landmark":
        print "1"
        lm = next((x.value for x in parameters if x.key=='landmark'), None)
        print "2"
        if lm:
            print "3"
            if lm in lm_known:
                receiptResponse.verbal_response, receiptResponse.context = \
                    ["Heading to landmark now", "navigating_to_landmark"]
                receiptResponse.timestamp       = str(datetime.now())
                vocalResponse.publish(receiptResponse)
                response, context = navToLandmark(lm_known, lm)
            else:
                response = "I dont't know where that is!"
        else:
            response = "I need a landmark!"

    # Create landmark 
    elif request.action_id == "build_landmark":
        lm = next((x.value for x in parameters if x.key=='landmark'), None)
        if lm:
            response, context = constructLandmark(lm_known, lm, currentGlobalPose, landmarkMarker)
   
    # Nav to coordinates
    elif request.action_id == "navigate_to_coordinate":
        print "4"
        pos = Point()
        print parameters
        for x in parameters:
            print "keyyy "
            print x.key
            print "valueee "
            print x.value
            print type(x)
            print type(x.value)
        coordinates = next((x for x in parameters if x.key=='coordinate'), None)
        if coordinates:
            coords = ast.literal_eval(coordinates.value)
            try:
                pos.x = int(coords['x'])
                pos.y = int(coords['y'])
                pos.z = 0
                receiptResponse.verbal_response, receiptResponse.context = ["Heading to the coordinates!", "navigating_to_coordinates"]
                vocalResponse.publish(receiptResponse)
                move_base.goto(pos, 0)
                response = "I completed my coordinate navigation!"
            except ValueError:
                response = "Are you sure those were numbers?"
        
        else:
            response = "I didn't hear any coordinates?"



    completionResponse.verbal_response = response
    completionResponse.context         = context
    completionResponse.timestamp       = str(datetime.now())
    vocalResponse.publish(completionResponse)


class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("have move base")
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
