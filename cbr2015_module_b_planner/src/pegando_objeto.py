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

	rospy.logwarn("Color: %s", objeto)
        client.send_goal(goal)
        client.wait_for_result()
	rospy.logwarn(client.get_result())
	'''
	'''
	while client.get != True:
		rospy.logwarn("Esperando completar grab_puck")
	PRECISA VERIFICAR SE A PECA FOI REALMENTE ENTREGUE
	'''
	atualizaArea(area, Objetos.NONE)
