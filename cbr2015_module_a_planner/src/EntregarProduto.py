import rospy
from std_msgs.msg import String
from robotino_leds.srv import *
from robotino_motion.msg import AlignAction
from robotino_motion.msg import AlignGoal
from robotino_motion.msg import StorePuckAction
from robotino_motion.msg import StorePuckGoal
import actionlib
from actionlib import SimpleActionClient

def entregarProduto(nome):
	#stop_transportation = rospy.ServiceProxy('stop_transportation', Trigger)
	#resp = stop_transportation()
	#rospy.loginfo(resp.message)
	client = actionlib.SimpleActionClient('store_puck', StorePuckAction)
        client.wait_for_server()

	rospy.logwarn("entregando produto")
	goal = StorePuckGoal()
	goal.mode = 1
	if(nome == "Deposito1"):
		goal.store_number = 0
	elif(nome == "Deposito2"):
		goal.store_number = 1
	elif(nome == "Deposito3"):
		goal.store_number = 2
	elif(nome == "Casa"):
		goal.store_number = 5

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

	rospy.logwarn("Entregando Produto")

   	#sinalize_end = rospy.ServiceProxy('sinalize_end', Trigger)
	#resp = sinalize_end() 

	'''client = actionlib.SimpleActionClient('align', AlignAction)
        client.wait_for_server()

	goal.alignment_mode = 3
	goal.distance_mode = 1

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()'''
