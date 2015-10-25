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

        client.send_goal(goal)
        client.wait_for_result()
	'''
	atualizaArea(area, Objetos.NONE)
