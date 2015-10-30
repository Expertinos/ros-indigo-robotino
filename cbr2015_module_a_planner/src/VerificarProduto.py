import rospy
from robotino_motion.msg import *
from robotino_vision.srv import *
from robotino_vision.msg import *

def verificarProduto(produto):
	#alinhar = rospy.ServiceProxy('align', Align)
	#resp = alinhar()

	produtos = []
	produtos.append(produto)

	contem = rospy.ServiceProxy('contain_in_list', ContainInList)
	resp = contem(produtos)

	rospy.logwarn('verificando produto')	

	return resp
