import actionlib
import rospy
import yaml
import sys
import atexit
from datetime import datetime

from math import sin, cos

from nav_msgs.msg import Odometry
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from spoken_interaction.msg import VerbalRequest, VerbalResponse, KeyValue


# Move base using navigation stack
def storeLoc(globLoc):
    global x, y 
    x = globLoc.pose.pose.position.x
    y = globLoc.pose.pose.position.y


def verbalReqHandler(request):
    print "I heard you say: %s" % request.phrase
    resp = VerbalResponse()
    resp.timestamp  = str(datetime.now())
    resp.request_id = request.timestamp
    

    if request.action_id == "navigate_to_landmark":
        lm = None
        for p in request.params:
            if p.key == 'landmark':
                lm = p.value
        if lm in landmarks:
            response = "Heading to " + lm + " at " + str(landmarks[lm][0]) + " " + str(landmarks[lm][1])
            move_base.goto(landmarks[lm][0], landmarks[lm][1], 1.0)
            resp.context = "heading to landmark"
        else:
            response = "I dont't know where that is!"
    
    '''if action == "navigate_to_coordinates":
        move_base.goto(long(xc), long(yc), long(tc))
    '''
    if request.action_id == "create_landmark":
        for p in request.params:
            if p.key == 'landmark':
                lm = p.value
        landmarks[lm] = (x,y)
        response = "I'll remember that " + lm +" is at " +  str(int(x)) + " and " + str(int(y))

    resp.verbal_response = response

    vocalResponse.publish(resp)


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


def handle_exit():
    f = open(sys.argv[1], 'w+')
    yaml.dump(landmarks, f)
    print "dumping the following int %s" % sys.argv[1]
    print landmarks
    f.close()

if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")    
    x, y = 0, 0
    if sys.argv == 1:
        rospy.loginfo("no landmark path provided")
        exit(0)
    landmarksFile = open(sys.argv[1], 'r')
    landmarks = yaml.load(landmarksFile)
    if not landmarks:
        landmarks = {}
    
    print "opened a file"
    print landmarks


    landmarksFile.close()
    
    try:
        globLoc = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, storeLoc)
        vocalComman = rospy.Subscriber("verbal_input", VerbalRequest, verbalReqHandler)
        vocalResponse = rospy.Publisher("verbal_response", VerbalResponse, queue_size = 10)

        # Make sure sim time is working
        while not rospy.Time.now():
            pass
        rospy.spin()
        # Setup clients
        move_base = MoveBaseClient()

    finally:
        handle_exit()
