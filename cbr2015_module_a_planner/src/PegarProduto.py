import rospy
from robotino_leds.srv import *
from enum import *

def pegarProduto(produto):
	transport_product = rospy.ServiceProxy('transport_product', TransportProduct)

	resp = transport_product(produto)
        rospy.loginfo('pegando produto')
