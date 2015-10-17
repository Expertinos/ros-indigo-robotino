import rospy
from std_msgs.msg import String
from robotino_leds.srv import *


def entregarProduto():
	#stop_transportation = rospy.ServiceProxy('stop_transportation', Trigger)
	#resp = stop_transportation()
	#rospy.loginfo(resp.message)
	rospy.logwarn("Entregando Produto")

   	sinalize_end = rospy.ServiceProxy('sinalize_end', Trigger)
	resp = sinalize_end() 

