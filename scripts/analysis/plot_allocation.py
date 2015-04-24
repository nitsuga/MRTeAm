#!/usr/bin/env python

import getopt
import glob, os, pickle, re, sys
import pprint
import rosbag
from collections import defaultdict

# Stats/plotting libraries
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

# Some global constants
mechanisms = ['RR', 'OSI', 'SSI', 'PSI']

start_configs = ['clustered', 'distributed']
#start_configs = ['distributed']

#point_configs = ['A','B','C','D','E']
#task_files = ['brooklyn_tasks_A.txt', 'brooklyn_tasks_C.txt', 'brooklyn_tasks_E.txt']
task_files = ['tasks_A.txt']

# stroke_colors = { 'robot_1': [0.369, 0.380, 0.773],
#                   'robot_2': [0.043, 0.063, 0.424],
#                   'robot_3': [0.176, 0.659, 0.000]
# }

stroke_colors = { 'robot_1': [1.0, 0.0, 0.0],
                  'robot_2': [0.0, 1.0, 0.0],
                  'robot_3': [0.0, 0.0, 1.0] }

robot_names = [ 'robot_1',
                'robot_2',
                'robot_3' ]

FILENAME_TEMPL='allocation-{0}.png'

class Experiment(object):
    def __init__(self, bag=None, mechanism=None, start_config=None, task_file=None):
        self.bag = bag
        self.mechanism = mechanism
        self.start_config = start_config
        self.task_file = task_file

def usage():
    print("Usage: {0}: <path_to_bag_file(s)>".format(sys.argv[0]))

def main(argv):
    if len(argv) < 1:
        usage()
        sys.exit(1)

    experiments = []
    exp_by_mechanism = defaultdict(list)
    exp_by_start_config = defaultdict(list)
    exp_by_task_file = defaultdict(list)

    dt_re = re.compile('(.*)\.bag')

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in argv:
        bag_paths.extend(glob.glob(path_arg))

    for i,bag_path in enumerate(bag_paths):
        print("Reading {0}".format(bag_path))

        bag = None
        try:
            bag = rosbag.Bag(bag_path)
        except:
            print("Couldn't open {0} for reading!".format(bag_path))
            continue

        bag_filename = os.path.basename(bag_path)
        (map, start_config, mechanism, task_file, remainder) = bag_filename.split('__')
        dt_match = dt_re.search(remainder)
        
        exp = Experiment(bag, mechanism, start_config, task_file)

        exp_by_mechanism[mechanism].append(exp)
        exp_by_start_config[start_config].append(exp)
        exp_by_task_file[task_file].append(exp)
        
    # ['A', 'C', 'E']
    for task_file in task_files:

        # Set plot title
        plt.title("Task Allocation, Task File '{0}' (Simulation)".format(task_file))

        #plt.tick_params(axis='y', which='major', labelsize=10)
        #plt.tick_params(axis='x', which='major', labelsize=8)

        # Position of the current allocation 'bar'
        x_pos = 0

        #grid = np.random.rand(8, len(experiments), 3)
        grid = np.random.rand(9, 80, 3)
        #grid = np.random.rand(8, 20, 3)

        for start_config in start_configs:
            for mechanism in mechanisms:

                exps = set(exp_by_start_config[start_config]).intersection(set(exp_by_mechanism[mechanism])).intersection(set(exp_by_task_file[task_file]))

                #for exp in sorted(exps, key=lambda e: int(e.run_number))[0:10]:
                #for exp in exps:
                for exp in list(exps)[0:10]:
                    print "{0}, {1}, {2}".format(start_config,
                                                 mechanism,
                                                 task_file)

                    # For each robot
                    # for role in sorted(exp.roles.keys()):
                    #     robot_name = exp.roles[role]
                    #     robot = exp.robots_by_name[robot_name]

                    #     # For each task that that robot won...
                    #     for task in robot.tasks.values():
                    #         # Record it in our grid
                    #         print "{0} won point {1}".format(role,int(task.auction_id)+1)
                    #         grid[task.auction_id][x_pos] = stroke_colors[role]

                    run_msgs = defaultdict(list)
                    for topic,msg,msg_time in exp.bag.read_messages('/tasks/award'):
                        print "{0} won task {1}".format(msg.robot_id,msg.tasks[0].task.task_id)
                        #grid[int(msg.tasks[0].task.task_id)-1][x_pos] = stroke_colors[msg.robot_id]

                        stroke_color = stroke_colors[msg.robot_id]
                        y_pos = int(msg.tasks[0].task.task_id)-1
                        grid[y_pos][x_pos] = stroke_color

                        #print "y_pos=={0}, x_pos=={1}, stroke_color=={2}".format(y_pos,
                        #                                                         x_pos,
                        #                                                         stroke_color)

                    x_pos += 1


        plt.imshow(grid[::-1], interpolation='none', extent=[0,80,0,9])
        plt.axes().set_aspect(9)

        # Set x, y limits (range)
        plt.ylim(0,9)
        plt.xlim(0,80)
        
        # Tick locations, labels, and sizes    
        yticks = plt.getp(plt.gca(), 'yticklines')
        plt.setp(yticks, 'linewidth', 0)
        plt.yticks([y + 0.5 for y in range(0,9)],
                   ['Point {0}'.format(y) for y in range(1,9)])

        xticks = plt.getp(plt.gca(), 'xticklines')
        plt.setp(xticks, 'linewidth', 0)
        plt.xticks([x + 5 for x in range(0,80,10)],
                   [ 'RR-C', 'OSI-C', 'SSI-C', 'PSI-C',
                     'RR-D', 'OSI-D', 'SSI-D', 'PSI-D'])

        # Grid lines (manually drawn)
        for i in range(0,80):
            plt.axvline(x=i, color='k', linewidth=0.1, solid_capstyle='butt')

        for i in range(0,80,10):
            plt.axvline(x=i, color='k', linewidth=0.5, solid_capstyle='butt')

        for i in range(9):
            plt.axhline(y=i, color='k', linewidth=0.5, solid_capstyle='butt')

        # Legend: robot colors
        ax = plt.subplot(111)
        legend_keys = []
        legend_labels = []
        for robot_num in range(1,4):
            legend_keys.append(plt.Rectangle((0,0), 1, 1, fc=stroke_colors['robot_%d' % (robot_num)]))
            legend_labels.append('Robot %d' % robot_num)
        
        lgd = ax.legend(legend_keys, legend_labels, loc='upper center',
                        bbox_to_anchor=(0.5, -0.05), fancybox=True, ncol=3, fontsize='small')
        
        plt.savefig(FILENAME_TEMPL.format(task_file),
                    dpi=300, bbox_extra_artists=(lgd,), bbox_inches='tight')
        plt.close()
    
if __name__ == '__main__':
    main(sys.argv[1:])

