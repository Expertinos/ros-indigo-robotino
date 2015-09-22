import rospy

def verificarProduto():
	transport_product = rospy.ServiceProxy('align', Align)

	resp = transport_product(produto)
	rospy.loginfo('verificando produto')
