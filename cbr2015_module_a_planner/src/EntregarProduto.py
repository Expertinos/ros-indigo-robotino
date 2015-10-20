import rospy
from std_msgs.msg import String
from robotino_leds.srv import *
from robotino_motion.msg import AlignAction
from robotino_motion.msg import AlignGoal
import actionlib
from actionlib import SimpleActionClient

def entregarProduto():
	#stop_transportation = rospy.ServiceProxy('stop_transportation', Trigger)
	#resp = stop_transportation()
	#rospy.loginfo(resp.message)
	'''client = actionlib.SimpleActionClient('align', AlignAction)
        client.wait_for_server()

	goal = AlignGoal()
	goal.alignment_mode = 2
	goal.distance_mode = 1

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()'''

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
