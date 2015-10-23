import rospy
from robotino_leds.srv import Sinalize

def ligarLed(pedidos):

	pedidos_piscar = []
	pedidos_piscar.append(pedidos[0])
	pedidos_piscar.append(0)
	pedidos_piscar.append(pedidos[1])
	pedidos_piscar.append(0)
	pedidos_piscar.append(pedidos[2])

	get_products_list = rospy.ServiceProxy('sinalize', Sinalize)
	resp = get_products_list(1, pedidos, 1, 0.5)

	rospy.logwarn("ligando led")
