#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from enum import *
global buffer_empty
global success
global cont

buffer_empty = True
success = False
cont = 0

# define state IndoParaArea
class IndoParaArea(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['chegou'],
				input_keys=['area'],
				output_keys=['indo_area'])

    def execute(self, userdata):
	userdata.indo_area = userdata.area
	rospy.logdebug('Cheguei')
	return 'chegou'

# define state EstouNaArea
class EstouNaArea(smach.State):   
    def __init__(self):
        smach.State.__init__(self, outcomes=['pegar_obj', 'deixar_obj'],
				input_keys=['area', 'objeto'],
				output_keys=['prox_area'])

    def execute(self, userdata):	
	if userdata.area == AreasSMORG.PRATDES:
		rospy.logdebug('Estou na Prateleira Desejada')
		if userdata.objeto == ObjetosSMORG.OBJDES and buffer_empty == True:
			userdata.prox_area = AreasSMORG.CASA
			rospy.logdebug('Estou na Prateleira Desejada com Objeto DESEJADO na primeira passada')
			success = True
			return 'deixar_obj'
		userdata.prox_area = AreasSMORG.BUFFER
		if userdata.objeto != ObjetosSMORG.OBJDES and buffer_empty == True:
			#userdata.prox_objeto = ObjetosSMORG.OBJAUX
			rospy.logdebug('Estou na Prateleira Desejada com Objeto AUXILIAR')
			return 'pegar_obj'
		if userdata.objeto == ObjetosSMORG.OBJDES and buffer_empty == False:
			rospy.logdebug('Estou na Prateleira Desejada com Objeto DESEJADO')
			return 'deixar_obj'

	elif userdata.area == AreasSMORG.BUFFER:
		rospy.logdebug('Estou no Buffer')
		userdata.prox_area = AreasSMORG.PRATAUX
		if buffer_empty == True:
			#userdata.prox_objeto = ObjetosSMORG.OBJAUX
			rospy.logdebug('Estou no Buffer e vou deixar o Objeto AUXILIAR')
			return 'deixar_obj'
		if buffer_empty == False:
			rospy.logdebug('Estou no Buffer e vou pegar o Objeto AUXILIAR')
			return 'pegar_obj'

	elif userdata.area == AreasSMORG.PRATAUX:

		rospy.logdebug('Estou na Prateleira Auxiliar')
		if userdata.objeto == ObjetosSMORG.OBJDES and buffer_empty == False:
			userdata.prox_area = AreasSMORG.PRATDES
			rospy.logdebug('Estou na Prateleira Auxiliar e vou pegar o Objeto DESEJADO')
			return 'pegar_obj'
		if userdata.objeto != ObjetosSMORG.OBJDES and buffer_empty == True:
			userdata.prox_area = AreasSMORG.CASA
			rospy.logdebug('Estou na Prateleira Auxiliar e vou deixar o Objeto AUXILIAR e ir pra casa')
			success = True
			return 'deixar_obj'
	userdata.prox_area = AreasSMORG.CASA
	rospy.logerr('Deu algum erro, vou deixar o Objeto e ir pra casa')
	success = False
	return 'deixar_obj'

# define state PegandoObjeto
class PegandoObjeto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pegou'],
				input_keys=['area', 'objeto'],
				output_keys=['prox_area'])

    def execute(self, userdata):
	global cont
	global buffer_empty
	userdata.prox_area = userdata.area
	rospy.logdebug('Peguei')
	if cont == 2:
		buffer_empty = True
	return 'pegou'

# define state DeixandoObjeto
class DeixandoObjeto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['deixou', 'finaliza'],
				input_keys=['area'],
				output_keys=['prox_area', 'prox_objeto'])

    def execute(self, userdata):
	global cont
	global buffer_empty
	userdata.prox_area = userdata.area
	if userdata.area == AreasSMORG.CASA:
		rospy.logdebug('Deixei e vou finalizar a prova')
		return 'finaliza'
	rospy.logdebug('Deixei e vou para a proxima area')	
	if cont == 1:
		userdata.prox_objeto = ObjetosSMORG.OBJAUX
		buffer_empty = False
		cont += 1
	if cont == 0:
		rospy.logdebug('Estou aqui')
		userdata.prox_objeto = ObjetosSMORG.OBJDES
		buffer_empty = False
		cont += 1	
	return 'deixou'
