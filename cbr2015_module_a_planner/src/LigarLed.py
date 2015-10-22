import rospy
from robotino_leds.srv import Sinalize

def ligarLed(pedidos):
	get_products_list = rospy.ServiceProxy('sinalize', Sinalize)
	resp = get_products_list(1, pedidos, 1, 1)

	rospy.logwarn("ligando led")
