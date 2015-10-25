#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from enum import *
global areas
global objeto
global terminou
global cores
from indo_para_area import *
from verificando_objeto import *
from ligando_leds import *

objeto = Objetos.NONE
areas = [Areas.A3, Areas.A1, Areas.A2, Areas.A4, Areas.B2, Areas.B4, Areas.B3, Areas.B1, Areas.CASA]
cores = [1, 2, 3, 4]
i = 0
terminou = False

# define state EstouNaAreaScan
class EstouNaAreaScan(smach.State):   
    def __init__(self):
        smach.State.__init__(self, outcomes=['verificar_obj', 'finaliza_scan', 'comeca_scan'],
				input_keys=['area'],
				output_keys=['prox_area'])

    def execute(self, userdata):
	global terminou
	global areas
	if userdata.area[0] == Areas.CASA[0] and terminou:
		rospy.logwarn('Terminei de verificar os Objetos')
		return 'finaliza_scan'
	elif userdata.area[0] == Areas.CASA[0]:
		userdata.prox_area = areas.pop(0)
		return 'comeca_scan'
	rospy.logwarn('Vou verificar Objeto')
	return 'verificar_obj'

# define state VerificandoObjeto ---> VISION para ver qual o objeto
class VerificandoObjeto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['verificou'],
				input_keys=['area'],
				output_keys=['prox_area'])

    def execute(self, userdata):
	global areas
	global objeto
	global i
	global terminou
	rospy.logwarn('Objeto Desatualizado: %s', userdata.area)
	
	if i == 0:
		objeto = Objetos.VERMELHO_UM
	if i == 1:
		objeto = Objetos.AZUL_TRES
	if i == 2:
		objeto = Objetos.AZUL_UM
	if i == 3:
		objeto = Objetos.VERMELHO_CINCO
	if i == 4:
		objeto = Objetos.AZUL_TRES
	if i == 5:
		objeto = Objetos.VERMELHO_TRES
	if i == 6:
		objeto = Objetos.AZUL_CINCO
	if i == 7:
		objeto = Objetos.VERMELHO_TRES
	i += 1
	atualizaArea(userdata.area,objeto)

	#verificandoArea(userdata.area)
	if areas[0] == Areas.CASA:
		ligandoLeds(cores, True)
		rospy.logwarn('Atualizei Objeto: %s, Terminou = %s', userdata.area, terminou)
		userdata.prox_area = areas.pop(0)
		terminou = True
		return 'verificou'
	rospy.logwarn('Atualizei Objeto: %s, Terminou = %s', userdata.area, terminou)
	userdata.prox_area = areas.pop(0)
	return 'verificou'
