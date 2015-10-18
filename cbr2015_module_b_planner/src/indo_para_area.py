from enum import *
#def indoParaArea(objeto):	
import rospy
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import Quaternion
from tf import transformations
import tf
import actionlib
from actionlib import SimpleActionClient

def indoParaArea(area, seq):
	#send_goal
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Waits until the action server has started up and started
        # listening for goals.

	goal = MoveBaseGoal()

	q = transformations.quaternion_from_euler(0, 0, area[3])
	quat = Quaternion(*q)

	goal.target_pose.header.seq = seq
	goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = area[1]
	goal.target_pose.pose.position.y = area[2]
	goal.target_pose.pose.orientation = quat

        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

	rospy.logwarn("Cheguei na Area: %s (X: %s, Y: %s)", area[0], area[1], area[2])
