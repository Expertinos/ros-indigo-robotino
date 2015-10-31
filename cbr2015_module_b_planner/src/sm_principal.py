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

	rospy.logwarn('INICIO')
	return 'inicio'

# define state Inicio
class Fim(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fim'])

    def execute(self, userdata):
	rospy.logwarn('FIM')	
	return 'fim'
