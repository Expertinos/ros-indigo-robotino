import rospy
from robotino_motion.srv import *
from robotino_vision.srv import *

def verificarProduto(pedidos):
	#alinhar = rospy.ServiceProxy('align', Align)
	#resp = alinhar()

	contem = rospy.ServiceProxy('contain_in_list', ContainInList)
	resp = contem(pedidos)

	rospy.logwarn('verificando produto')	

	return resp
