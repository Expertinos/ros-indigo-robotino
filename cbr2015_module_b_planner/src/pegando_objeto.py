import rospy
from robotino_leds.srv import *
from enum import *
from robotino_motion.msg import GrabPuckGoal
from robotino_motion.msg import GrabPuckAction
import actionlib
from actionlib import SimpleActionClient

def pegandoObjeto(area, objeto):
	'''
	rospy.logwarn('pegando objeto '+str(objeto))

	client = actionlib.SimpleActionClient('grab_puck', GrabPuckAction)
        client.wait_for_server()

	goal = GrabPuckGoal()
	goal.color = objeto

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

	#transport = rospy.ServiceProxy('transport_product', TransportProduct)
	#resp = transport(objeto)
	'''
	atualizaArea(area, Objetos.NONE)
