#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from enum import *
from enum import areaOrganizada
from pegando_objeto import *
from deixando_objeto import *
from indo_para_area import *
from ligando_leds import *
global buffer_empty
global success
global objeto
global terminou
global cores
global prox_area_aux
global prox_area
global area_verifica
global areas_ogz
global new_order
prox_area_aux = []
prox_area = []
area_verifica = AreasOrganizadas.A1

areas_ogz = [AreasOrganizadas.A1, AreasOrganizadas.A2, AreasOrganizadas.A3, AreasOrganizadas.A4, AreasOrganizadas.B1, AreasOrganizadas.B2, AreasOrganizadas.B3, AreasOrganizadas.B4]
areas = [Areas.A1, Areas.A2, Areas.A3, Areas.A4, Areas.B1, Areas.B2, Areas.B3, Areas.B4, Areas.CASA]

cores = [1, 2, 3, 4]

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
	global cores
	global terminou
	seq += 1
	rospy.logwarn('Area: %s', userdata.area[0])
	
	if userdata.area[0] == Areas.CASA:
		ligandoLeds(cores, True)
		indoParaArea(userdata.area, seq)
		desligandoLeds()
		return 'chegou'
	
	indoParaArea(userdata.area, seq)
	rospy.logwarn('Cheguei')
	return 'chegou'

# define state EstouNaArea
class EstouNaArea(smach.State):   
    def __init__(self):
        smach.State.__init__(self, outcomes=['pegar_obj', 'deixar_obj', 'fim_org', 'comeca_org'],
				input_keys=['area_atual', 'area_des', 'area_aux'],
				output_keys=['prox_area', 'area_parc'])

    def execute(self, userdata):
	global objeto
	global terminou
	global areas
	global prox_area_aux
	global prox_area
	rospy.logwarn("Area Atual: %s, Area Desejada: %s, Area Auxiliar: %s", userdata.area_atual, userdata.area_des, userdata.area_aux)
	if userdata.area_atual[0] == Areas.CASA[0]:	
		terminou = True
		return 'fim_org'
	if userdata.area_atual[0] == userdata.area_des[0]:
		rospy.logwarn('Estou na Area Desejada com o objeto %s, buffer %s', userdata.area_atual[4], buffer_empty)
		if not areaOrganizada(userdata.area_atual, objeto) and not areaComObjDesejado(userdata.area_des, userdata.area_aux):
			success = True
			terminou = True
			userdata.prox_area = Areas.CASA
			return 'comeca_org'
		if areaOrganizada(userdata.area_atual, objeto) and buffer_empty:			
			rospy.logwarn('Estou na Prateleira Desejada com Objeto DESEJADO na primeira passada')
			success = True
			terminou = True			
			userdata.prox_area = Areas.CASA
			return 'comeca_org'
		if not areaOrganizada(userdata.area_atual, objeto) and buffer_empty:
			rospy.logwarn('Estou na Prateleira Desejada com Objeto AUXILIAR')
			if len(userdata.area_atual[4]) == 0:
				userdata.prox_area = Areas.CASA
				success = True
				terminou = True
				return 'comeca_org'
			if userdata.area_aux == Areas.CASA:
				userdata.prox_area = Areas.CASA
				success = True
				terminou = True
				return 'comeca_org'
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
	pegandoObjeto(userdata.area_atual, objeto[1])
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
	rospy.logwarn('Area Atual :%s na Area Auxiliar: %s', userdata.area_atual, userdata.area_aux)
	rospy.logwarn('Deixei :%s na Area:%s', objeto, userdata.area_atual)
	objeto = Objetos.NONE
	if userdata.area_atual[0] == userdata.area_aux[0]:
		rospy.logwarn('Deixei e vou voltar para casa')
		return 'deixou'
	buffer_empty = False
	rospy.logwarn('Deixei e vou para a proxima area')
	return 'deixou'

# define state LendoPostes
class LendoPostes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['leitura_realizada', 'finaliza_prova'],
				input_keys=['area_atual'],
				output_keys=['prox_area', 'nova_area_des', 'nova_area_aux'])

    def execute(self, userdata):
	global terminou
	global prox_area
	global areas
	global new_order
	'''
	while new_order == False:
		rospy.logwarn("Esperando comando para iniciar")
	'''
	if userdata.area_atual[0] == Areas.CASA[0] and not terminou:
		rospy.logwarn('Estou em casa e vou comecar a ler os postes')
		prox_area = areas.pop(0)
		ondeIr()				
		userdata.prox_area = prox_area
		userdata.nova_area_des = prox_area
		novaAreaAux(prox_area)
		userdata.nova_area_aux = prox_area_aux
		#ligandoLeds(sinalizaLeitura(prox_area), terminou)
		return 'leitura_realizada'
	if userdata.area_atual[0] == Areas.CASA[0] and terminou:		
		if prox_area[0] == Areas.CASA[0]:
			rospy.logwarn('Finalizei a organizacao')
			rospy.logwarn('Como ficaram Areas A: %s, %s, %s, %s', Areas.A1, Areas.A2, Areas.A3, Areas.A4)
			rospy.logwarn('Como ficaram Areas B: %s, %s, %s, %s', Areas.B1, Areas.B2, Areas.B3, Areas.B4)
			return 'finaliza_prova'
		prox_area = areas.pop(0)
		ondeIr()
		userdata.prox_area = prox_area
		userdata.nova_area_des = prox_area
		novaAreaAux(prox_area)
		userdata.nova_area_aux = prox_area_aux
		rospy.logwarn('Acabei de organizar mais um e vou ler o prox pedido, %s', prox_area)
		terminou = False
		return 'leitura_realizada'

def novaAreaAux(area_des):
	global prox_area_aux
	global areas_ogz
	global area_verifica
	atualizaAreasOgz()
	for  i in range (0, 6):
		i += 1
		if area_des[0] == area_verifica[0]:
			if area_verifica[1] == Areas.A1[4]:
				if Areas.A1[4] != AreasOrganizadas.A1[1]:
					prox_area_aux = Areas.A1
					return
			elif area_verifica[1] == Areas.A2[4]:
				if Areas.A2[4] != AreasOrganizadas.A2[1]:
					prox_area_aux = Areas.A2		
					return	
			elif area_verifica[1] == Areas.A3[4]:
				if Areas.A3[4] != AreasOrganizadas.A3[1]:
					prox_area_aux = Areas.A3		
					return	
			elif area_verifica[1] == Areas.A4[4]:
				if Areas.A4[4] != AreasOrganizadas.A4[1]:
					prox_area_aux = Areas.A4		
					return	
			elif area_verifica[1] == Areas.B1[4]:
				if Areas.B1[4] != AreasOrganizadas.B1[1]:
					prox_area_aux = Areas.B1		
					return	
			elif area_verifica[1] == Areas.B2[4]:
				if Areas.B2[4] != AreasOrganizadas.B2[1]:
					prox_area_aux = Areas.B2		
					return	
			elif area_verifica[1] == Areas.B3[4]:
				if Areas.B3[4] != AreasOrganizadas.B3[1]:
					prox_area_aux = Areas.B3		
					return	
			elif area_verifica[1] == Areas.B4[4]:
				if Areas.B4[4] != AreasOrganizadas.B4[1]:
					prox_area_aux = Areas.B4		
					return
			else:
				prox_area_aux = Areas.CASA
				prox_area_aux[4] = Objetos.NONE
				return
		area_verifica = areas_ogz.pop(0)
	area_verifica = AreasOrganizadas.A1

def atualizaAreasOgz():
	global areas_ogz
	areas_ogz = [AreasOrganizadas.A1, AreasOrganizadas.A2, AreasOrganizadas.A3, AreasOrganizadas.A4, AreasOrganizadas.B1, AreasOrganizadas.B2, AreasOrganizadas.B3, AreasOrganizadas.B4]

def ondeIr():
	global areas
	global prox_area
	global terminou
	for  i in range (0, 6):
		i += 1
		if prox_area[0] == Areas.CASA[0]:
			return
		elif areaOrganizada(prox_area, objeto):
			ligandoLeds(sinalizaLeitura(prox_area), terminou)
			ligandoLeds(cores, True)	
			rospy.logwarn('Area ja organizada, bora pra Proxima')		
			prox_area = areas.pop(0)
		else:
			ligandoLeds(sinalizaLeitura(prox_area), terminou)
			return prox_area
