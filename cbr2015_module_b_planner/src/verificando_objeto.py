from enum import *
import rospy
from robotino_vision.srv import *

def verificandoObjeto(area, color):
	objeto = ['', 0, 0]
	findObjects = rospy.ServiceProxy('find_objects', FindObjects)
	resp = findObjects(color, True, 0)
	if len(resp.number_of_markers) == 1:
		objeto[1] = color
		objeto[2] = resp.number_of_markers[0]
		rospy.logwarn('objeto %s', objeto)
		if objeto[1] == Cores.AZUL:
			if objeto[2] == 1:
				objeto[0] = 'AZUL_UM'
			elif objeto[2] == 3:
				objeto[0] = 'AZUL_TRES'
			elif objeto[2] == 5:
				objeto[0] = 'AZUL_CINCO'
			atualizaArea(area, objeto)
			return True
		elif objeto[1] == Cores.VERMELHO:
			if objeto[2] == 1:
				objeto[0] = 'VERMELHO_UM'
			elif objeto[2] == 3:
				objeto[0] = 'VERMELHO_TRES'
			elif objeto[2] == 5:
				objeto[0] = 'VERMELHO_CINCO'
			atualizaArea(area, objeto)
			return
		else:
			objeto = Objetos.NONE
			atualizaArea(area,objeto)

def verificandoArea(area):
	color = Cores.AZUL
	if verificandoObjeto(area, color):
		return
	else: 
		color = Cores.VERMELHO
		verificandoObjeto(area, color)
