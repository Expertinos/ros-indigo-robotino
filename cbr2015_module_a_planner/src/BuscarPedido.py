import rospy
from std_msgs.msg import String

def buscarPedido(pub):
	rospy.loginfo('buscando pedido')
	string = String()
	string.data = 'buscar_pedido|x|y'
	pub.publish(string)
