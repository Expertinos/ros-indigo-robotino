from enum import *
import rospy
from robotino_vision.srv import *

def lendoPostes():
	get_lamp_posts = rospy.ServiceProxy('get_lamp_posts', GetLampPosts)
	resp = get_lamp_posts()
	if resp.success:
		if resp.left.green:
			if resp.right.green:
				return Areas.A1
			elif resp.right.yellow:
				return Areas.A4
			elif resp.right.red:
				return Areas.B1
		elif resp.left.yellow:
			if resp.right.yellow:
				return Areas.A2
			elif resp.right.red:
				return Areas.B2
			elif resp.right.green:
				return Areas.B3
		elif resp.left.red:
			if resp.right.red:
				return Areas.A3
			elif resp.right.green:
				return Areas.B4
	else:
		return Areas.CASA
