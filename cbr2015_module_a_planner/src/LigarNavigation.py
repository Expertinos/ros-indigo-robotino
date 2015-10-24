import rospy
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import Quaternion
from robotino_motion.msg import AlignAction
from robotino_motion.msg import AlignGoal
from tf import transformations
import tf
import actionlib
from actionlib import SimpleActionClient
from geometry_msgs.msg import Twist

def atualizaCmdVel(vel):
	global vel_x
	global vel_y
	global ang_x
	global ang_y

	vel_x = vel.linear.x
	vel_y = vel.linear.y
	ang_x = vel.angular.x
	ang_y = vel.angular.y


def ligarNavigation(area, seq, nome):
	#send_goal
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Waits until the action server has started up and started
        # listening for goals.

	goal = MoveBaseGoal()

	q = transformations.quaternion_from_euler(0, 0, area[2])
	quat = Quaternion(*q)

	goal.target_pose.header.seq = seq
	goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = area[0]
	goal.target_pose.pose.position.y = area[1]
	goal.target_pose.pose.orientation = quat

	#pub = rospy.Publisher("move_base_simple/goal", PoseStamped)
	#pub.publish(goal.target_pose)

        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal)

	'''global vel_x
	global vel_y
	global ang_x
	global ang_y

	rospy.logwarn(client.get_state())
	while(client.get_state() == 0 or client.get_state() == 1):
		rospy.logwarn(client.get_state())
		rospy.logwarn("dentro do while")
		tempo1 = rospy.get_time()
		tempo2 = rospy.get_time()
		while(tempo2-tempo1 < 4 or vel_x != 0 or vel_y != 0 or ang_x != 0 or ang_y != 0):
			tempo2 = rospy.get_time()

		if(tempo2 - tempo1 >= 4):
			twist = Twist()
			twist.linear.x = 0.5
			pub = rospy.Publisher('cmd_vel', Twist)
			for i in range(0, 30):
				pub.publish(twist)

			twist.linear.x = 0
			pub.publish(twist)
	rospy.logwarn("passou o while")'''
        # Waits for the server to finish performing the action.
        client.wait_for_result()

	rospy.logwarn("Cheguei no pose "+ str(area[0]) +" "+ str(area[1]) +"Area = "+nome)

	if nome == "Pedidos":
		rospy.logwarn("Vou alinhar a direita")
		client = actionlib.SimpleActionClient('align', AlignAction)
		client.wait_for_server()

		goal = AlignGoal()

		goal.alignment_mode = 1
		goal.distance_mode = 0

		# Sends the goal to the action server.
		client.send_goal(goal)

		# Waits for the server to finish performing the action.
		client.wait_for_result()

	if nome == "Casa":
		rospy.logwarn("Vou alinhar a direita")
		client = actionlib.SimpleActionClient('align', AlignAction)
		client.wait_for_server()

		goal = AlignGoal()

		goal.alignment_mode = 1
		goal.distance_mode = 1

		# Sends the goal to the action server.
		client.send_goal(goal)

		# Waits for the server to finish performing the action.
		client.wait_for_result()
	
		rospy.logwarn("Vou alinhar atras")
		client = actionlib.SimpleActionClient('align', AlignAction)
		client.wait_for_server()

		goal.alignment_mode = 3
		goal.distance_mode = 1

		# Sends the goal to the action server.
		client.send_goal(goal)

		# Waits for the server to finish performing the action.
		client.wait_for_result()

	

	'''if client.getState() == actionlib.SimpleClientGoalState.SUCCEEDED:
		ROS_INFO("Cheguei no pose "+ str(area[0]) + str(area[1]));
	else:
		ROS_INFO("BUGOU!!!!");'''
