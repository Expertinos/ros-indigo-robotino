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

def ligarNavigation(area, seq):
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

	#pub = rospy.Publisher("move_base_simple/goal", PoseStamped)
	#pub.publish(goal.target_pose)

	print "antes do wait"
        client.wait_for_server()
	print "depois do wait"

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

	print "Cheguei no pose "+ str(area[1]) +" "+ str(area[2])
	'''if client.getState() == actionlib.SimpleClientGoalState.SUCCEEDED:
		ROS_INFO("Cheguei no pose "+ str(area[0]) + str(area[1]));
	else:
		ROS_INFO("BUGOU!!!!");'''
