from enum import *
import rospy
import time
from robotino_motion.msg import GrabPuckGoal
from robotino_motion.msg import GrabPuckAction
import actionlib
from actionlib import SimpleActionClient

def pegandoObjeto(area):
	'''	
	#send_goal
	client = actionlib.SimpleActionClient('grab_puck', GrabPuckAction)

        # Waits until the action server has started up and started
        # listening for goals.

	goal = GrabPuckGoal()

	goal.color = area[4][1]

        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

	rospy.logwarn("Peguei objeto %s na Area %s", area[4][0], area[0])
	'''
	atualizaArea(area, Objetos.NONE)
