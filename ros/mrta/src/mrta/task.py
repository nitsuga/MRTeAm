"""task.py


This module defines the Task class used in multirobot experiments.


Eric Schneider <eric.schneider@liverpool.ac.uk>

"""

class Point:
    """
    A simple class that represents a point in 3d space. It has just three (float)
    members: x, y, and z.
    """
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class Task(object):

    def __init__(self, task_id=None, type=None):

        # The instantiator is responsible for coming up with a
        # task_id that is unique/valid/etc., according to whatever
        # properties are required.
        self.task_id = task_id

        # We only work with location-based, "sweep" tasks for now.
        self.type = type

        # A list of task_ids that this task depends on
        self.depends = []
        
        # Number of robots are needed to complete the task
        self.num_robots = 1

        # Time required to complete the task, in seconds
        self.duration = 0

        # Has the task been awarded (assigned) to a robot?
        self.awarded = False

        # Has the task been completed?
        self.completed = False

class SensorSweepTask(Task):

    def __init__(self, task_id=None, x=0.0, y=0.0, z=0.0, _num_robots=1, _duration=0):
        super(SensorSweepTask, self).__init__(task_id, type='SENSOR_SWEEP')

        self.location = Point(x, y, z)
        self.num_robots = _num_robots
        self.duration = _duration
