import rospy
from robotino_leds.srv import *
from enum import *

def pegarProduto(produto):
	transport_product = rospy.ServiceProxy('transport_product', TransportProduct)

        rospy.logwarn('pegando produto '+str(produto))

	resp = transport_product(produto)
