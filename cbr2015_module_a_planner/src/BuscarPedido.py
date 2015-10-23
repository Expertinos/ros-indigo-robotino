import rospy
import time
import sys
from std_msgs.msg import String
from robotino_vision.srv import *
from robotino_motion.msg import AlignAction
from robotino_motion.msg import AlignGoal 
import actionlib
from actionlib import SimpleActionClient

def buscarPedido(pub):

	'''rospy.logwarn("Vou alinhar a direita")
	client = actionlib.SimpleActionClient('align', AlignAction)
	client.wait_for_server()

	goal = AlignGoal()

	goal.alignment_mode = 1
	goal.distance_mode = 1

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	client.wait_for_result()'''
		

	rospy.logwarn('buscando pedido')

	'''valores = []
	for i in range(0, 5):
		valores.append(0)

	for i in range (0, 10):'''

	get_products_list = rospy.ServiceProxy('get_objects_list', GetObjectsList)
	resp = get_products_list()
	
	'''rospy.logwarn("\n\n Leitura "+str(i))
	for p in resp.objects:
		rospy.logwarn(p)

	for r in resp.objects:
		valores[r] += 1

	time.sleep(0.2)

	resposta = []
	for i in range(0, len(valores)):
		if round(valores[i]/10) >= 1:
			resposta.append(i) 
	'''
	
	for i in resp.objects:
		rospy.logwarn(i)
		
	return resp.objects

	'''string = String()
	string.data = 'buscar_pedido|x|y'
	pub.publish(string)'''
