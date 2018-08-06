# All 3D vectors are referenced in the fingertip core center

Header header					# stamp
geometry_msgs/Vector3 resultant_force  		# Resultant Force
geometry_msgs/Vector3 resultant_torque      	# Resultant Torque
float64[] hull_force_dist			# distances from force hull surface to origin in each octant
float64[] hull_torque_dist			# distances from force hull surface to origin in each octant
float64 hull_volume_force			# volume of force hull. basically the grasp quality. unitless.
float64 hull_volume_torque			# volume of torque hull

