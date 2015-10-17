#!/usr/bin/env python

import roslib; roslib.load_manifest('cbr2015_module_b_planner')
import rospy
import smach
import smach_ros
from sm_principal import *
from sm_org import *

global areas
global areas_desorganizadas
global areas_organizadas
global casa
global buff

# main
def main():


	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['end'])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('INICIO', Inicio(), 
				       transitions={'inicio':'SUB'})
		# Create a SMACH state machine ORGANIZA
		sm_org = smach.StateMachine(outcomes=['fim'])
		sm_org.userdata.sm_org_area = AreasSMORG.PRATDES
		sm_org.userdata.sm_org_objeto = ObjetosSMORG.OBJAUX 

		# Open sm_org
		with sm_org:
			# Add states to the sm_org
			smach.StateMachine.add('INDOPARAAREA', IndoParaArea(), 
					       transitions={'chegou':'ESTOUNAAREA'},
					       remapping={'area':'sm_org_area', 'objeto':'sm_org_objeto',
							  'indo_area':'sm_org_area'})
			smach.StateMachine.add('ESTOUNAAREA', EstouNaArea(), 
					       transitions={'pegar_obj':'PEGANDOOBJETO', 'deixar_obj':'DEIXANDOOBJETO'},
					       remapping={'area':'sm_org_area', 'objeto':'sm_org_objeto',
							  'prox_area':'sm_org_area'})
			smach.StateMachine.add('PEGANDOOBJETO', PegandoObjeto(), 
					       transitions={'pegou':'INDOPARAAREA'},
					       remapping={'area':'sm_org_area', 'objeto':'sm_org_objeto',
							  'prox_area':'sm_org_area'})
			smach.StateMachine.add('DEIXANDOOBJETO', DeixandoObjeto(), 
					       transitions={'deixou':'INDOPARAAREA', 'finaliza':'fim'},
					       remapping={'area':'sm_org_area',
							  'prox_area':'sm_org_area', 'prox_objeto':'sm_org_objeto'})
		smach.StateMachine.add('SUB', sm_org,
				transitions={'fim':'FIM'})	
		smach.StateMachine.add('FIM',Fim(), 
				transitions={'fim':'end'})

	# Execute SMACH plan
	outcome = sm.execute()

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
	rospy.init_node('cbr2015_module_b_node', log_level = rospy.DEBUG)
    	rospy.loginfo("cbr2015_modula_a node is up and running!!!")
	main()
