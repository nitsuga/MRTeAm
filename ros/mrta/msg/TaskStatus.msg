std_msgs/Header header		    # Message header
string robot_id			        # Unique robot identifier
string task_id			        # Unique task identifier
uint8 status
uint8 MOVING				= 0 # Robot has begun to travel toward the task
uint8 PAUSE					= 1 # Paused (e.g., to avoid a collision)
uint8 RESUME				= 2 # Resuming from a PAUSE
uint8 ARRIVED				= 3 # Robot has arrived at the task site
uint8 BEGIN					= 4 # Robot has begun executing the task
uint8 SUCCESS				= 5 # Task successfully executed
uint8 FAILURE				= 6 # Task failed
uint8 ALL_TASKS_COMPLETE 	= 7 # All tasks have been completed