string task_id		   # Unique task identifier
string[] depends	   # Ids of other tasks that must be completed first
string type		   # Currently just 'SENSOR_SWEEP'
uint8 num_robots_required # Number of robots needed to complete this task
float32 seconds_required  # Time (in seconds) required to 'execute' the task