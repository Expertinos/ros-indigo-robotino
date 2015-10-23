from enum import *
import rospy
from robotino_vision.srv import *
from ligando_leds import *

def verificandoObjeto(area, color):
	objeto = ['', 0, 0]
	findObjects = rospy.ServiceProxy('find_objects', FindObjects)
	resp = findObjects(color, True, 0)
	objeto[1] = color
	objeto[2] = obtemMaior(resp.number_of_markers)
	rospy.logwarn('objeto %s', objeto)
	if objeto[1] == Cores.AZUL:
		if objeto[2] == 0 or objeto[2] == 1:
			objeto[0] = 'AZUL_UM'
			if objeto[2] == 0:
				objeto[2] = 1
		elif objeto[2] == 2 or objeto[2] == 3:
			objeto[0] = 'AZUL_TRES'
			if objeto[2] == 2:
				objeto[2] = 3
		elif objeto[2] == 4 or objeto[2] == 5:
			objeto[0] = 'AZUL_CINCO'
			if objeto[2] == 4:
				objeto[2] = 5
		atualizaArea(area, objeto)
		ligandoLeds2([color, color], False, objeto[2])
		return
	elif objeto[1] == Cores.VERMELHO:
		if objeto[2] == 0 or objeto[2] == 1:
			objeto[0] = 'VERMELHO_UM'
			if objeto[2] == 0:
				objeto[2] = 1
		elif objeto[2] == 2 or objeto[2] == 3:
			objeto[0] = 'VERMELHO_TRES'
			if objeto[2] == 2:
				objeto[2] = 3
		elif objeto[2] == 4 or objeto[2] == 5:
			objeto[0] = 'VERMELHO_CINCO'
			if objeto[2] == 4:
				objeto[2] = 5
		atualizaArea(area, objeto)
		ligandoLeds2([color, color], False, objeto[2])
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

def obtemMaior(markers):
	global final_marker
	final_marker = 0
	for marker in markers:
		if marker > final_marker:
			final_marker = marker
	return final_marker
