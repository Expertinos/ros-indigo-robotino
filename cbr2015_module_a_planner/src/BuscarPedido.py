import rospy
import time
import sys
from std_msgs.msg import String
from robotino_vision.srv import *
from robotino_motion.msg import AlignAction
from robotino_motion.msg import AlignGoal
from robotino_motion.msg import ReadOrderAction
from robotino_motion.msg import ReadOrderGoal
import actionlib
from actionlib import SimpleActionClient

def buscarPedido(pub):
		

	rospy.logwarn('buscando pedido')

	'''valores = []
	for i in range(0, 5):
		valores.append(0)
'''

	#get_products_list = rospy.ServiceProxy('get_objects_list', GetObjectsList)
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

	resposta = resp.colors
	rospy.logwarn("leitura dos pedidos")
	rospy.logwarn(resposta)
		
	return resposta'''
	return [3,1,4]
