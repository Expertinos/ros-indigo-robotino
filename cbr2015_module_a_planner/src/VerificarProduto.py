import rospy
from robotino_motion.srv import *
from robotino_vision.srv import *

def verificarProduto(produto):
	#alinhar = rospy.ServiceProxy('align', Align)
	#resp = alinhar()

	produtos = []
	produtos.append(produto)

	contem = rospy.ServiceProxy('contain_in_list', ContainInList)
	resp = contem(produtos)

	rospy.logwarn('verificando produto')	

	return resp
