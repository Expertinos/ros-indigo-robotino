import rospy
from std_msgs.msg import String
from robotino_vision.srv import *

def buscarPedido(pub):

	get_products_list = rospy.ServiceProxy('get_products_list', GetProductsList)

	resp = get_products_list()

	rospy.logwarn('buscando pedido')
	
	for p in resp.products:
		rospy.logwarn(p)
	
	return resp.products

	'''string = String()
	string.data = 'buscar_pedido|x|y'
	pub.publish(string)'''
