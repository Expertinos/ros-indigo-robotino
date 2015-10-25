#!/usr/bin/env python

import roslib; roslib.load_manifest('cbr2015_module_b_planner')
import rospy
import smach
import smach_ros
from sm_principal import *
from sm_org import *
from sm_scan import *
from enum import *
from std_srvs.srv import Empty

global areas_desorganizadas
global areas_organizadas

# main
def main():
	global new_order
	new_order = False


	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['end'])

	# Open the container
	with sm:
		smach.StateMachine.add('INICIO', Inicio(), 
				       transitions={'inicio':'SCAN'})
		
		# Create the sub SMACH state machine SCAN
		sm_scan = smach.StateMachine(outcomes=['areas_verificadas'])
		sm_scan.userdata.sm_scan_area = Areas.CASA
				
		# Open sm_org
		with sm_scan:
			# Add states to the sm_org
			smach.StateMachine.add('INDOPARAAREASCAN', IndoParaArea(), 
					       transitions={'chegou':'ESTOUNAAREASCAN'},
					       remapping={'area':'sm_scan_area'})
			smach.StateMachine.add('ESTOUNAAREASCAN', EstouNaAreaScan(), 
					       transitions={'verificar_obj':'VERIFICANDOOBJETO', 'finaliza_scan':'areas_verificadas', 									'comeca_scan':'INDOPARAAREASCAN'},
					       remapping={'area':'sm_scan_area',
								'prox_area':'sm_scan_area'})
			smach.StateMachine.add('VERIFICANDOOBJETO', VerificandoObjeto(), 
					       transitions={'verificou':'INDOPARAAREASCAN'},
					       remapping={'area':'sm_scan_area',
								'prox_area':'sm_scan_area'})
		smach.StateMachine.add('SCAN', sm_scan,
				transitions={'areas_verificadas':'ORG'})


		# Create the sub SMACH state machine ORGANIZA
		sm_org = smach.StateMachine(outcomes=['fim'])
		sm_org.userdata.sm_org_area_des = Areas.CASA
		sm_org.userdata.sm_org_area_aux = Areas.CASA
		sm_org.userdata.sm_org_area_atual = Areas.CASA
		sm_org.userdata.sm_org_area_parc = Areas.CASA

		# Open sm_org
		with sm_org:
			# Add states to the sm_org
			smach.StateMachine.add('LENDOPOSTES', LendoPostes(),
				transitions={'leitura_realizada':'INDOPARAAREA','finaliza_prova':'fim'},
				remapping={'area_atual':'sm_org_area_atual',
						'prox_area':'sm_org_area_atual', 'nova_area_des':'sm_org_area_des', 								'nova_area_aux':'sm_org_area_aux'})
			smach.StateMachine.add('INDOPARAAREA', IndoParaArea(), 
					       transitions={'chegou':'ESTOUNAAREA'},
					       remapping={'area':'sm_org_area_atual'})
			smach.StateMachine.add('ESTOUNAAREA', EstouNaArea(), 
					       transitions={'pegar_obj':'PEGANDOOBJETO', 'deixar_obj':'DEIXANDOOBJETO', 								'fim_org':'LENDOPOSTES', 'comeca_org':'INDOPARAAREA'},
					       remapping={'area_des':'sm_org_area_des', 'area_aux':'sm_org_area_aux', 									'area_atual':'sm_org_area_atual',
							  'prox_area':'sm_org_area_atual', 'area_parc':'sm_org_area_parc'})
			smach.StateMachine.add('PEGANDOOBJETO', PegandoObjeto(), 
					       transitions={'pegou':'INDOPARAAREA'},
					       remapping={'area_atual':'sm_org_area_parc'})
			smach.StateMachine.add('DEIXANDOOBJETO', DeixandoObjeto(), 
					       transitions={'deixou':'INDOPARAAREA'},
					       remapping={'area_atual':'sm_org_area_parc', 'area_aux':'sm_org_area_aux'})
		smach.StateMachine.add('ORG', sm_org,
				transitions={'fim':'FIM'})	
		smach.StateMachine.add('FIM',Fim(), 
				transitions={'fim':'end'})

	# Execute SMACH plan
	outcome = sm.execute()

def seta_order(req):
	global new_order
	new_order = True

def printAreas():
	for area in areas_desorganizadas:
		rospy.logwarn('%s:%s', area[0], area[4])

def readParameters():
	if not rospy.has_param("/cbr2015_module_b_node/area_a1_x"):
		rospy.logfatal('There are no areas parameters. You must call this node through roslaunch.')
		rospy.signal_shutdown('No parameters.')
		return
	
	global areas_organizadas
	global areas_desorganizadas
	global buff
	global casa

	areas_organizadas = []
	areas_desorganizadas = []

	Areas.A1[1] = rospy.get_param("/cbr2015_module_b_node/area_a1_x")
	Areas.A1[2] = rospy.get_param("/cbr2015_module_b_node/area_a1_y")
	Areas.A1[3] = rospy.get_param("/cbr2015_module_b_node/area_a1_orientation")

	Areas.A2[1] = rospy.get_param("/cbr2015_module_b_node/area_a2_x")
	Areas.A2[2] = rospy.get_param("/cbr2015_module_b_node/area_a2_y")
	Areas.A2[3] = rospy.get_param("/cbr2015_module_b_node/area_a2_orientation")

	Areas.A3[1] = rospy.get_param("/cbr2015_module_b_node/area_a3_x")
	Areas.A3[2] = rospy.get_param("/cbr2015_module_b_node/area_a3_y")
	Areas.A3[3] = rospy.get_param("/cbr2015_module_b_node/area_a3_orientation")

	Areas.A4[1] = rospy.get_param("/cbr2015_module_b_node/area_a4_x")
	Areas.A4[2] = rospy.get_param("/cbr2015_module_b_node/area_a4_y")
	Areas.A4[3] = rospy.get_param("/cbr2015_module_b_node/area_a4_orientation")

	Areas.B1[1] = rospy.get_param("/cbr2015_module_b_node/area_b1_x")
	Areas.B1[2] = rospy.get_param("/cbr2015_module_b_node/area_b1_y")
	Areas.B1[3] = rospy.get_param("/cbr2015_module_b_node/area_b1_orientation")

	Areas.B2[1] = rospy.get_param("/cbr2015_module_b_node/area_b2_x")
	Areas.B2[2] = rospy.get_param("/cbr2015_module_b_node/area_b2_y")
	Areas.B2[3] = rospy.get_param("/cbr2015_module_b_node/area_b2_orientation")

	Areas.B3[1] = rospy.get_param("/cbr2015_module_b_node/area_b3_x")
	Areas.B3[2] = rospy.get_param("/cbr2015_module_b_node/area_b3_y")
	Areas.B3[3] = rospy.get_param("/cbr2015_module_b_node/area_b3_orientation")

	Areas.B4[1] = rospy.get_param("/cbr2015_module_b_node/area_b4_x")
	Areas.B4[2] = rospy.get_param("/cbr2015_module_b_node/area_b4_y")
	Areas.B4[3] = rospy.get_param("/cbr2015_module_b_node/area_b4_orientation")

	Areas.CASA[1] = rospy.get_param("/cbr2015_module_b_node/casa_x")
	Areas.CASA[2] = rospy.get_param("/cbr2015_module_b_node/casa_y")
	Areas.CASA[3] = rospy.get_param("/cbr2015_module_b_node/casa_orientation")

	Areas.BUFFER[1] = rospy.get_param("/cbr2015_module_b_node/buffer_x")
	Areas.BUFFER[2] = rospy.get_param("/cbr2015_module_b_node/buffer_y")
	Areas.BUFFER[3] = rospy.get_param("/cbr2015_module_b_node/buffer_orientation")

	areas_desorganizadas.append(Areas.A1)
	areas_desorganizadas.append(Areas.A2)
	areas_desorganizadas.append(Areas.A3)
	areas_desorganizadas.append(Areas.A4)
	areas_desorganizadas.append(Areas.B1)
	areas_desorganizadas.append(Areas.B2)
	areas_desorganizadas.append(Areas.B3)
	areas_desorganizadas.append(Areas.B4)
	casa = Areas.CASA
	buff = Areas.BUFFER


if __name__ == '__main__':
	rospy.init_node('cbr2015_module_b_node')# log_level = rospy.DEBUG)
    	rospy.logwarn("cbr2015_modula_b node is up and running!!!")
	s = rospy.Service('new_order', Empty, seta_order)
	main()
