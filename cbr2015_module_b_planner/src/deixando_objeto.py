import rospy
#from robotino_leds.srv import *
from enum import *

def deixandoObjeto(area, objeto):
	if area[0] == Areas.A1[0]:
		Areas.A1[4] = objeto
		return
	elif area[0] == Areas.A2[0]:
		Areas.A2[4] = objeto
		return	
	elif area[0] == Areas.A3[0]:
		Areas.A3[4] = objeto
		return
	elif area[0] == Areas.A4[0]:
		Areas.A4[4] = objeto
		return
	elif area[0] == Areas.B1[0]:
		Areas.B1[4] = objeto
		return
	elif area[0] == Areas.B2[0]:
		Areas.B2[4] = objeto
		return
	elif area[0] == Areas.B3[0]:
		Areas.B3[4] = objeto
		return
	elif area[0] == Areas.B4[0]:
		Areas.B4[4] = objeto
		return
	elif area[0] == Areas.BUFFER[0]:
		Areas.BUFFER[4] = objeto
		return
