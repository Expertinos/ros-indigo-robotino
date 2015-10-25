import rospy
from robotino_vision.srv import *

def verificaDeposito():
	get_products_list = rospy.ServiceProxy('get_objects_list', GetObjectsList)
	resp = get_products_list()

	rospy.logwarn('verificando deposito')	

	return resp.objects
