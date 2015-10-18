#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from enum import *
from enum import areaOrganizada
global buffer_empty
global success
global prat_des
global prat_aux
global obj_des
global obj_aux
global objeto
global terminou
from pegando_objeto import *
from deixando_objeto import *
from indo_para_area import *

buffer_empty = True
success = False
terminou = False
objeto = Objetos.NONE

seq = 0

# define state IndoParaArea
class IndoParaArea(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['chegou'],
				input_keys=['area'])

    def execute(self, userdata):
	global seq
	seq += 1
	#indoParaArea(userdata.area, seq)
	rospy.logwarn('Cheguei')
	return 'chegou'

# define state EstouNaArea
class EstouNaArea(smach.State):   
    def __init__(self):
        smach.State.__init__(self, outcomes=['pegar_obj', 'deixar_obj', 'fim_org'],
				input_keys=['area_atual', 'area_des', 'area_aux'],
				output_keys=['prox_area', 'area_parc'])

    def execute(self, userdata):
	global objeto
	global terminou
	rospy.logwarn("Area Atual: %s, Area Desejada: %s, Area Auxiliar: %s", userdata.area_atual, userdata.area_des, userdata.area_aux)
	if userdata.area_atual[0] == Areas.CASA[0] and terminou:
		rospy.logwarn('Finalizei e estou em casa')
		return 'fim_org'
	if userdata.area_atual[0] == userdata.area_des[0]:
		rospy.logwarn('Estou na Area Desejada')
		if areaOrganizada(userdata.area_atual, objeto) and buffer_empty:			
			rospy.logwarn('Estou na Prateleira Desejada com Objeto DESEJADO na primeira passada')
			success = True
			terminou = True
			userdata.area_parc = userdata.area_atual			
			userdata.prox_area = Areas.CASA
			return 'deixar_obj'
		if not areaOrganizada(userdata.area_atual, objeto) and buffer_empty:
			rospy.logwarn('Estou na Prateleira Desejada com Objeto AUXILIAR')
			userdata.area_parc = userdata.area_atual			
			userdata.prox_area = Areas.BUFFER
			return 'pegar_obj'
		if areaOrganizada(userdata.area_atual, objeto) and not buffer_empty:	
			rospy.logwarn('Estou na Prateleira Desejada com Objeto DESEJADO')
			userdata.area_parc = userdata.area_atual			
			userdata.prox_area = Areas.BUFFER
			return 'deixar_obj'
	elif userdata.area_atual[0] == Areas.BUFFER[0]:
		rospy.logwarn('Estou no Buffer')
		if buffer_empty == True:
			rospy.logwarn('Estou no Buffer e vou deixar o Objeto AUXILIAR')
			userdata.area_parc = userdata.area_atual			
			userdata.prox_area = userdata.area_aux
			return 'deixar_obj'
		if buffer_empty == False:
			rospy.logwarn('Estou no Buffer e vou pegar o Objeto AUXILIAR')
			userdata.area_parc = userdata.area_atual			
			userdata.prox_area = userdata.area_aux
			return 'pegar_obj'
	elif userdata.area_atual[0] == userdata.area_aux[0]:
		rospy.logwarn('Estou na Prateleira Auxiliar, Buffer: %s', buffer_empty)
		if areaComObjDesejado(userdata.area_des, userdata.area_atual) and not buffer_empty:
			rospy.logwarn('Estou na Prateleira Auxiliar e vou pegar o Objeto DESEJADO')
			rospy.logwarn('%s', userdata.area_atual)	
			userdata.area_parc = userdata.area_atual			
			userdata.prox_area = userdata.area_des			
			return 'pegar_obj'
		if not areaComObjDesejado(userdata.area_des, userdata.area_atual) and buffer_empty:
			rospy.logwarn('Estou na Prateleira Auxiliar e vou deixar o Objeto AUXILIAR e ir pra casa')
			success = True
			terminou = True			
			userdata.area_parc = userdata.area_atual			
			userdata.prox_area = Areas.CASA
			return 'deixar_obj'
	rospy.logerr('Deu algum erro, vou deixar o Objeto e ir pra casa')
	success = False
	terminou = True
	userdata.prox_area = Areas.CASA
	return 'deixar_obj'

# define state PegandoObjeto
class PegandoObjeto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pegou'],
				input_keys=['area_atual'])

    def execute(self, userdata):
	global buffer_empty
	global objeto
	rospy.logwarn('Peguei e vou para a proxima area')
	objeto = userdata.area_atual[4]
	pegandoObjeto(userdata.area_atual)
	rospy.logwarn("%s", userdata.area_atual)
	if userdata.area_atual[0] == Areas.BUFFER[0]:
		buffer_empty = True
	return 'pegou'

# define state DeixandoObjeto
class DeixandoObjeto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['deixou'],
				input_keys=['area_atual', 'area_aux'])

    def execute(self, userdata):
	global buffer_empty
	global objeto
	deixandoObjeto(userdata.area_atual, objeto)
	if userdata.area_atual[0] == userdata.area_aux[0]:
		rospy.logwarn('Deixei e vou voltar para casa')
		return 'deixou'
	buffer_empty = False
	rospy.logwarn('Deixei e vou para a proxima area')
	return 'deixou'
