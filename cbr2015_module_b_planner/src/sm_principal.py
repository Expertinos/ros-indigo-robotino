#!/usr/bin/env python

import roslib; roslib.load_manifest('cbr2015_module_b_planner')
import rospy
import smach
import smach_ros
from enum import *
from cbr2015_module_b_planner import readParameters
from cbr2015_module_b_planner import printAreas

# define state Inicio
class Inicio(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['inicio'])

    def execute(self, userdata):

	readParameters() 
	'''#BAGUNCA PROVISORIA
	Areas.A1[4] = Objetos.AZUL_UM
	Areas.A2[4] = Objetos.AZUL_TRES
	Areas.A3[4] = Objetos.AZUL_CINCO
	Areas.A4[4] = Objetos.VERMELHO_CINCO
	Areas.B1[4] = Objetos.VERMELHO_TRES
	Areas.B2[4] = Objetos.VERMELHO_UM
	Areas.B3[4] = Objetos.VERMELHO_TRES
	Areas.B4[4] = Objetos.AZUL_CINCO
	#######################################
	'''

	rospy.logwarn('INICIO')
	return 'inicio'

# define state Inicio
class Fim(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fim'])

    def execute(self, userdata):
	rospy.logwarn('FIM')	
	return 'fim'
