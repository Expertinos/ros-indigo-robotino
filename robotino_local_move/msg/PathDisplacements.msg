float32 move_x		# in meters
float32 move_y		# in meters
float32 rotation	# in degrees

uint8 movementType	
# 0 -> Translational Movement (move_phi should be equal to zero)
# 1 -> Rotational Movement (move_x and move_y should be equal to zero)
# 2 -> Translational and Rotational Moviment (at the same time)
# 3 -> Tangent Moviment (move_x must be iqual to move_y)
