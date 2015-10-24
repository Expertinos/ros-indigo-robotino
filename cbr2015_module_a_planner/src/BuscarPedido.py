import rospy
import time
import sys
from std_msgs.msg import String
from robotino_vision.srv import *
from robotino_motion.msg import AlignAction
from robotino_motion.msg import AlignGoal 
import actionlib
from actionlib import SimpleActionClient

def buscarPedido(pub):
		

	rospy.logwarn('buscando pedido')

	'''valores = []
	for i in range(0, 5):
		valores.append(0)
'''

	matriz = {}
	get_products_list = rospy.ServiceProxy('get_objects_list', GetObjectsList)
	i = 0
	rospy.logwarn("leitura dos pedidos")
	while i < 20:
		resp = get_products_list()
		aux = resp.objects
		if len(aux) == 3:
			rospy.logwarn(aux)
			if aux in matriz:
				valor = matriz[aux]
				valor =+ 1
				matriz[aux] = valor
			else:
				matriz[aux] = 1	
			i += 1
		time.sleep(0.1)
		
	
	'''rospy.logwarn("\n\n Leitura "+str(i))
	for p in resp.objects:
		rospy.logwarn(p)

	for r in resp.objects:
		valores[r] += 1'''

	time.sleep(0.2)
	
	sorted(matriz.items(), key=lambda x: (-x[1], x[0]))
	k = list(matriz.keys())
	resposta = k[0]
		

	'''for i in range(0, len(valores)):
		if round(valores[i]/10) >= 1:
			resposta.append(i) 
	
	resposta = []

	for i in resp.objects:
		if i not in resposta:
			resposta.append(i)
	'''
	rospy.logwarn("\n\nresposta")
	for i in resposta:
		rospy.logwarn(i)
		
	return resposta

	'''string = String()
	string.data = 'buscar_pedido|x|y'
	pub.publish(string)'''
