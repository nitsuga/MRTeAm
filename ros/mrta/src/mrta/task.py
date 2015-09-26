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

    def __init__(self, _task_id='1', _type=None, _num_robots=1, _duration=0, _depends=[]):

        # The instantiator is responsible for coming up with a
        # task_id that is unique/valid/etc.
        self.task_id = _task_id

        # We only work with location-based, "sweep" tasks for now.
        self.type = _type

        # A list of task_ids that this task depends on
        self.depends = []
        
        # Number of robots are needed to complete the task
        self.num_robots = _num_robots

        # Time required to complete the task, in seconds
        self.duration = _duration

        # Has the task been awarded (assigned) to a robot?
        self.awarded = False

        # Has the task been completed?
        self.completed = False

class SensorSweepTask(Task):

    def __init__(self, _task_id='1', x=0.0, y=0.0, z=0.0, _num_robots=1, _duration=0, _depends=[]):
        super(SensorSweepTask, self).__init__(_task_id, 'SENSOR_SWEEP', _num_robots, _duration, _depends)
        self.location = Point(x, y, z)
