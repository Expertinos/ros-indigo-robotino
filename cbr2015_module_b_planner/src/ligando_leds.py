from enum import *
import rospy
from robotino_leds.srv import *

def ligandoLeds(colors):
	sinalize = rospy.ServiceProxy('sinalize', 1, colors, -1, 1)
	resp = sinalize() 

def desligandoLeds():
	stop_sinalization = rospy.ServiceProxy('stop_sinalization')
	resp = stop_sinalization() 
