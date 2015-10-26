import rospy
from enum import *
from std_msgs.msg import String
from robotino_leds.srv import *
from robotino_motion.msg import StorePuckAction
from robotino_motion.msg import StorePuckGoal
import actionlib
from actionlib import SimpleActionClient

def deixandoObjeto(area, objeto):
	'''
	client = actionlib.SimpleActionClient('store_puck', StorePuckAction)
        client.wait_for_server()

	goal = StorePuckGoal()
	goal.mode = 0

        client.send_goal(goal)

        client.wait_for_result()
	'''
	rospy.logwarn("Entregando Produto")

	atualizaArea(area, objeto)
