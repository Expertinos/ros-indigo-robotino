from enum import *
import rospy
from robotino_vision.srv import *
from robotino_motion.msg import StorePuckAction
from robotino_motion.msg import StorePuckGoal

def lendoPostes(set_postes, seq_postes):
	#LENDO POSTES SETADO :/
	if set_postes < 4:
		return seq_postes[set_postes]
	else:
		return Areas.CASA
	'''
	get_lamp_posts = rospy.ServiceProxy('get_lamp_posts', GetLampPosts)
	resp = get_lamp_posts()
	if resp.success:
		if resp.left.green:
			if resp.right.green:
				return Areas.A1
			elif resp.right.yellow:
				return Areas.A4
			elif resp.right.red:
				return Areas.B1
		elif resp.left.yellow:
			if resp.right.yellow:
				return Areas.A2
			elif resp.right.red:
				return Areas.B2
			elif resp.right.green:
				return Areas.B3
		elif resp.left.red:
			if resp.right.red:
				return Areas.A3
			elif resp.right.green:
				return Areas.B4
	else:
		return Areas.CASA
	'''

def alinhaParaLeitura():
	'''
	client = actionlib.SimpleActionClient('store_puck', StorePuckAction)
        client.wait_for_server()

	goal = StorePuckGoal()
	goal.mode = 1
	goal.6
        client.send_goal(goal)
        client.wait_for_result()
	'''
	rospy.logwarn("Alinhei para leitura")

def alinhaVoltandoCasa():
	'''
	client = actionlib.SimpleActionClient('store_puck', StorePuckAction)
        client.wait_for_server()

	goal = StorePuckGoal()
	goal.mode = 1
	goal.5
        client.send_goal(goal)
        client.wait_for_result()
	'''
	rospy.logwarn("Alinhei de volta para casa")
