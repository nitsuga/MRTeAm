string robot_id			# id of the robot sending the message
geometry_msgs/Pose pose		# Current location of the robot
float64 cost	   		# Difference in cost between old and new (repaired)
				# plans, as estimated by the global planner