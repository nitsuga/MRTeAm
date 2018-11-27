#!/usr/bin/env python

import sys
import rospy
import rosbag
import signal
import pprint
import time
from datetime import datetime
import mrta.msg
from collections import defaultdict
from threading import Lock, Thread
from std_msgs.msg import String
import geometry_msgs.msg

pp = pprint.PrettyPrinter(indent=2)

mutex = Lock()
still_recording = True
time_stamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
# rosbag_name = 'exp_02__strand__PL-0__ARC_S03_tasks__full__%s.bag' %(time_stamp)
rosbag_name = '%s__%s.bag' % (sys.argv[1], time_stamp)
rec_bag = rosbag.Bag(rosbag_name, 'w')

experiment = tasks_stat = tasks_award = tasks_new = debug = tasks_announce = robot_1_amcl = robot_2_amcl = robot_3_amcl = None                  


def chatterbox(msg):
    mutex.acquire()
    rec_bag.write('/chatter', msg)
    mutex.release()

def exp(exp_msg):
    mutex.acquire()
    rec_bag.write('/experiment',exp_msg)
    mutex.release()
        
def task_status(task_msg):
    mutex.acquire()
    rec_bag.write('/tasks/status',task_msg)
    mutex.release() 
    
def task_award(task_msg):
    mutex.acquire()
    rec_bag.write('/tasks/award',task_msg)
    mutex.release() 
    
def task_new(task_msg):
    mutex.acquire()
    rec_bag.write('/tasks/new',task_msg)
    mutex.release() 
    
def debugging(debug_msg):
    mutex.acquire()
    rec_bag.write('/debug',debug_msg)
    mutex.release() 
    
def announce(announce_msg):
    mutex.acquire()
    rec_bag.write('/tasks/announce',announce_msg)
    mutex.release() 
    
def r1_pose_received(amcl_msg):
    mutex.acquire()
    rec_bag.write('/robot_1/amcl_pose',amcl_msg)
    mutex.release() 
    
def r2_pose_received(amcl_msg):
    mutex.acquire()
    rec_bag.write('/robot_2/amcl_pose',amcl_msg)
    mutex.release() 
    
def r3_pose_received(amcl_msg):
    mutex.acquire()
    rec_bag.write('/robot_3/amcl_pose',amcl_msg)
    mutex.release()


experiment = rospy.Subscriber('/experiment',
                               mrta.msg.ExperimentEvent,
                               exp)
                                  
tasks_stat = rospy.Subscriber('/tasks/status',
                               mrta.msg.TaskStatus,
                               task_status)

tasks_award = rospy.Subscriber('/tasks/award',
                                mrta.msg.TaskAward,
                                task_award)

tasks_new = rospy.Subscriber('/tasks/new',
                              mrta.msg.SensorSweepTask,
                              task_new)

debug = rospy.Subscriber('/debug',
                          mrta.msg.Debug,
                          debugging)
                                  
tasks_announce = rospy.Subscriber('/tasks/announce',
                             mrta.msg.AnnounceSensorSweep,
                             announce)
                                  
robot_1_amcl = rospy.Subscriber('/robot_1/amcl_pose',
                                 geometry_msgs.msg.PoseWithCovarianceStamped,
                                 r1_pose_received)
                                  
robot_2_amcl = rospy.Subscriber('/robot_2/amcl_pose',
                                 geometry_msgs.msg.PoseWithCovarianceStamped,
                                 r2_pose_received)

robot_3_amcl = rospy.Subscriber('/robot_3/amcl_pose',
                                 geometry_msgs.msg.PoseWithCovarianceStamped,
                                 r3_pose_received)
time.sleep(1) 

print(pp.pformat('Recording data to file with name: %s' %(rosbag_name))) 
print(pp.pformat('Subscribed to /experiment'))
print(pp.pformat('Subscribed to /tasks/status'))
print(pp.pformat('Subscribed to /tasks/award'))
print(pp.pformat('Subscribed to /tasks/new'))
print(pp.pformat('Subscribed to /debug'))
print(pp.pformat('Subscribed to /tasks/announce'))
print(pp.pformat('Subscribed to /robot_1/amcl_pose'))
print(pp.pformat('Subscribed to /robot_2/amcl_pose'))
print(pp.pformat('Subscribed to /robot_3/amcl_pose'))

def sig_handler(sig, frame):
    """ kill node and stop recording data when we get a SIGINT """
    if sig == signal.SIGINT:
        rec_bag.close()
        still_recording = False
        print("Shutting down and closing rosbag record...")
        sys.exit(0)

signal.signal(signal.SIGINT, sig_handler)

if __name__ == '__main__':
    while still_recording:
        node_name = 'recording'
        rospy.loginfo("Starting node '{0}'...".format(node_name))
        rospy.init_node(node_name)
        rospy.spin()
 
    

