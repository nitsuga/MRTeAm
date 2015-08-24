std_msgs/Header header		# Message header
string experiment_id		# Unique id of this experiment
uint8 event
uint8 BEGIN_EXPERIMENT	= 0
uint8 END_EXPERIMENT	= 1
uint8 BEGIN_ALLOCATION	= 2
uint8 END_ALLOCATION	= 3
uint8 BEGIN_EXECUTION	= 4
uint8 END_EXECUTION		= 5