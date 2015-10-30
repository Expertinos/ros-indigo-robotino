import rospy
from robotino_vision.srv import *
from actionlib import SimpleActionClient
import actionlib
from robotino_motion.msg import ReadOrderAction
from robotino_motion.msg import ReadOrderGoal

def verificaDeposito():
	'''client = actionlib.SimpleActionClient('read_order', ReadOrderAction)

        # Waits until the action server has started up and started
        # listening for goals.

	goal = ReadOrderGoal()

	goal.valid_colors = [1, 3, 4]
	goal.valid_number_of_objects = 3

	client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal)

        client.wait_for_result()
	resp = client.get_result()

	rospy.logwarn('verificando deposito')	

	return resp.colors'''
	return 0
