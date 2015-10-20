#!/usr/bin/env python

# Python libraries
import glob, os, sys, pickle, rosbag
from collections import defaultdict

# Stats/plotting libraries
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

# Some global constants
mechanisms = ['RR', 'OSI', 'SSI', 'PSI']
#start_configs = ['CLUSTERED', 'DISTRIBUTED']
start_configs = ['clustered', 'distributed']

#point_configs = ['A','B','C','D','E']
#point_configs = ['A','C','E']

#task_files = ['tasks_A.txt']
#task_files = ['TASC_scenario_5.txt']
#task_files = ['TASC_scenario_6.txt']
task_files = ['MR-CT-DA-scenario1.yaml']

robot_names = [ 'robot_1',
                'robot_2',
                'robot_3' ]

class Experiment(object):
    def __init__(self, bag=None, mechanism=None, start_config=None, task_file=None):
        self.bag = bag
        self.mechanism = mechanism
        self.start_config = start_config
        self.task_file = task_file

def plot_overall_stat(af=None, attr_name=None, title=None, y_label=None, out_filename=None, y_limit=None, plot_samples=False):
    print "Plotting {0}".format(out_filename)

    # A dict of <mechanism_name> => <list of values>
    stat_by_mechanism = defaultdict(list)

    # Populate said lists of values, by mechanism
    # for exp in experiments:
    #     # If exp.<attr_name> is an attribute, collect its value. If it's an
    #     # instance method, call it to obtain its value
    #     attr_value = getattr(exp, attr_name)
    #     if attr_value.__func__:
    #         stat_by_mechanism[exp.mechanism].append(attr_value.__call__())
    #     else:
    #         stat_by_mechanism[exp.mechanism].append(attr_value)

    plt.title(title)
    plt.ylabel(y_label)
    stat_means = []
    stat_errors = []

    for i,mechanism in enumerate(mechanisms):
        sample = af[af.MECHANISM==mechanism][attr_name]
        stat_means.append(np.mean(sample))
        stat_errors.append(stats.sem(sample))

        if plot_samples:
            # Plot sample values as red dots
            plt.plot([(i+.15) for x in sample], sample, 'r.')

    # x-coordinates of the bars
    x_pos = np.arange(len(mechanisms))
    # Plot bars with std errors
    plt.bar(x_pos, stat_means, yerr=stat_errors, capsize=8,
            color=[0.75,0.75,0.75], width=0.6, align='center')

    # Bar labels
    plt.tick_params(axis='x', which='major', labelsize=18)
    plt.xticks(x_pos, mechanisms)    
    ymin, ymax = plt.ylim()

    if y_limit:
        plt.ylim(0, y_limit)
    else:
        plt.ylim(0, ymax)

    plt.tight_layout()

    plt.savefig(out_filename)
    plt.close()

def plot_stacked_stats(af=None, attr1_name=None, attr2_name=None, title=None, y_label=None, out_filename=None, y_limit=None, plot_samples=False):
    print "Plotting {0}".format(out_filename)

    # A dict of <mechanism_name> => <list of values>
    stat_by_mechanism = defaultdict(list)

    # Populate said lists of values, by mechanism
    # for exp in experiments:
    #     # If exp.<attr_name> is an attribute, collect its value. If it's an
    #     # instance method, call it to obtain its value
    #     attr_value = getattr(exp, attr_name)
    #     if attr_value.__func__:
    #         stat_by_mechanism[exp.mechanism].append(attr_value.__call__())
    #     else:
    #         stat_by_mechanism[exp.mechanism].append(attr_value)

    plt.title(title)
    plt.ylabel(y_label)
    stat1_means = []
    stat2_means = []
    stat1_errors = []
    stat2_errors = []

    for i,mechanism in enumerate(mechanisms):
        sample1 = af[af.MECHANISM==mechanism][attr1_name]
        stat1_means.append(np.mean(sample1))
        stat1_errors.append(stats.sem(sample1))

        sample2 = af[af.MECHANISM==mechanism][attr2_name]
        stat2_means.append(np.mean(sample2))
        stat2_errors.append(stats.sem(sample2))

        if plot_samples:
            # Plot sample values as red dots
            plt.plot([(i+.15) for x in sample1], sample1, 'r.')
            plt.plot([(i+.15) for x in sample2], sample2, 'r.')

    # x-coordinates of the bars
    x_pos = np.arange(len(mechanisms))
    # Plot bars with std errors
    plt.bar(x_pos, stat1_means, yerr=stat1_errors, capsize=8,
            color=[0.5,0.5,0.5], width=0.6, align='center')

    plt.bar(x_pos, stat2_means, yerr=stat2_errors, capsize=8,
            color=[0.75,0.75,0.75], width=0.6, align='center',
            bottom=stat1_means)

    # Bar labels
    plt.tick_params(axis='x', which='major', labelsize=18)
    plt.xticks(x_pos, mechanisms)

    ymin, ymax = plt.ylim()

    if y_limit:
        plt.ylim(0,y_limit)
    else:
        plt.ylim(0, ymax)

    plt.tight_layout()

    plt.savefig(out_filename)
    plt.close()

def plot_per_robot_stat(experiments=[], attr_name=None, title=None, y_label=None, out_filename=None):
    print "Plotting {0}".format(out_filename)

    bar_width = 0.25
    x_start = 0.25
    x_pos = np.arange(x_start, len(mechanisms) + x_start)

    # A dict of <mechanism_name> => <list of values>
    stat_by_mechanism = { 'RR': defaultdict(list),
                          'OSI': defaultdict(list),
                          'SSI': defaultdict(list),
                          'PSI': defaultdict(list) }

    stat_by_role = { 'robot-1': defaultdict(list),
                     'robot-2': defaultdict(list),
                     'robot-3': defaultdict(list) }

    all_role_names = set()
    # Populate said lists of values, by mechanism
    for exp in experiments:
        for role_name,robot_name in sorted(exp.roles.items()):


            all_role_names.add(role_name)
            robot = exp.robots_by_name[robot_name]

            # If exp.<attr_name> is an attribute, collect its value. If it's an
            # instance method, call it to obtain its value
            # hax
            if attr_name == 'robot_travel_time' or attr_name == 'robot_idle_time':
                attr_value = getattr(exp, attr_name)
                stat_by_role[role_name][exp.mechanism].append(attr_value.__call__(robot_name))

            else:
                attr_value = getattr(robot, attr_name)

                if attr_value.__func__:
                    stat_by_role[role_name][exp.mechanism].append(attr_value.__call__())
                else:
                    stat_by_role[role_name][exp.mechanism].append(attr_value)

    all_role_names = sorted(all_role_names)

    plt.title(title)
    plt.ylabel(y_label)

    means_by_role = defaultdict(list)
    errors_by_role = defaultdict(list)
    
    for i,mechanism in enumerate(mechanisms):
        for j,role_name in enumerate(all_role_names):
#            print "{0}, {1}".format(mechanism, role_name)
            means_by_role[role_name].append(np.mean(stat_by_role[role_name][mechanism]))
            errors_by_role[role_name].append(stats.sem(stat_by_role[role_name][mechanism]))

#            print stat_by_role[role_name][mechanism]
            # Plot sample values as red dots
#            plt.plot([x + (bar_width*i) for x in x_pos],
#                     stat_by_role[role_name][mechanism], 'r.')

    bars = []
    # Plot bars with std errors
    for i,role_name in enumerate(all_role_names):
        grey_level = 0.25 + (i*0.25)
        bars.append(plt.bar([x + (bar_width*i) for x in x_pos],
                    means_by_role[role_name], yerr=errors_by_role[role_name],
                    color=[grey_level] * 3, width=bar_width))

    # Bar labels
    plt.xticks([x+0.33 for x in x_pos], mechanisms)    
    ymin, ymax = plt.ylim()
    plt.ylim(0, ymax+(ymax*0.2))
    plt.xlim(0, x_pos[-1] + 1)
    plt.legend( [bar[0] for bar in bars], all_role_names )

    plt.savefig(out_filename)
    plt.close()

            
def main(argv):

    aframe = pd.read_csv(argv[0])

    #### Stats for all point configs, grouped by start config { CLUSTERED, DISTRIBUTED }
    for start_config in start_configs:

        # #### Deliberation time
        # plot_title = 'Overall Deliberation Time: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'deliberation-overall-{0}.pdf'.format(start_config)
        # plot_overall_stat( experiments=by_start_config[start_config],
        #                    attr_name='deliberation_time',
        #                    title=plot_title,
        #                    y_label='Seconds',
        #                    out_filename=plot_filename )

        # #### Execution time
        # plot_title = 'Overall Execution Time: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'execution-overall-{0}.pdf'.format(start_config)
        # plot_overall_stat( experiments=by_start_config[start_config],
        #                    attr_name='execution_time',
        #                    title=plot_title,
        #                    y_label='Seconds',
        #                    out_filename=plot_filename )

        # #### Idle time
        # plot_title = 'Overall Idle Time: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'idle-overall-{0}.pdf'.format(start_config)        
        # plot_overall_stat( experiments=by_start_config[start_config],
        #                    attr_name='total_idle_time',
        #                    title=plot_title,
        #                    y_label='Seconds',
        #                    out_filename=plot_filename )

        # #### Delay time
        # plot_title = 'Overall Delay Time: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'delay-overall-{0}.pdf'.format(start_config)        
        # plot_overall_stat( experiments=by_start_config[start_config],
        #                    attr_name='total_delay_time',
        #                    title=plot_title,
        #                    y_label='Seconds',
        #                    out_filename=plot_filename )

        # #### Distance travelled
        # plot_title = 'Overall Distance Travelled: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'distance-overall-{0}.pdf'.format(start_config)        
        # plot_overall_stat( experiments=by_start_config[start_config],
        #                    attr_name='total_travel_distance',
        #                    title=plot_title,
        #                    y_label='cm',
        #                    out_filename=plot_filename )

        # #### Near collisions
        # plot_title = 'Overall Near Collisions: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'collisions-overall-{0}.pdf'.format(start_config)        
        # plot_overall_stat( experiments=by_start_config[start_config],
        #                    attr_name='total_collisions',
        #                    title=plot_title,
        #                    out_filename=plot_filename )

        # #### Distance travelled per robot
        # plot_title = 'Distance per Robot: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'distance-per-robot-{0}.pdf'.format(start_config)
        # plot_per_robot_stat( experiments=by_start_config[start_config],
        #                      attr_name='total_travel_distance',
        #                      title=plot_title,
        #                      y_label='cm',
        #                      out_filename=plot_filename )

        # #### Travel time per robot
        # plot_title = 'Travel time per Robot: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'travel-time-per-robot-{0}.pdf'.format(start_config)
        # plot_per_robot_stat( experiments=by_start_config[start_config],
        #                      attr_name='robot_travel_time',
        #                      title=plot_title,
        #                      y_label='Seconds',
        #                      out_filename=plot_filename )

        # #### Idle time per robot
        # plot_title = 'Idle time per Robot: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'idle-time-per-robot-{0}.pdf'.format(start_config)
        # plot_per_robot_stat( experiments=by_start_config[start_config],
        #                      attr_name='robot_idle_time',
        #                      title=plot_title,
        #                      y_label='Seconds',
        #                      out_filename=plot_filename )

        # #### Delay time per robot
        # plot_title = 'Delay time per Robot: (All point configs), {0} start'.format(start_config)
        # plot_filename = 'delay-per-robot-{0}.pdf'.format(start_config)
        # plot_per_robot_stat( experiments=by_start_config[start_config],
        #                      attr_name='total_delay_time',
        #                      title=plot_title,
        #                      y_label='Seconds',
        #                      out_filename=plot_filename )

        #### Stats by point config
        for task_file in task_files:

            af = aframe[aframe.START_CONFIG==start_config][aframe.TASK_FILE==task_file]

            task_file = task_file.replace('.txt','')

            # Total run time stacked: deliberation time + execution time
            #### Total run time
            plot_title = 'Total Run Time, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'stacked-runtime-{0}-{1}.pdf'.format(task_file, start_config)
            plot_stacked_stats( af = af,
                                attr1_name='DELIBERATION_TIME',
                                attr2_name='EXECUTION_PHASE_TIME',
                                title=plot_title,
                                y_label='Seconds',
                                out_filename=plot_filename)
#                                y_limit=450)

            #### Total run time
            plot_title = 'Total Run Time, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'run-{0}-{1}.pdf'.format(task_file, start_config)
            plot_overall_stat( af = af,
                               attr_name='TOTAL_RUN_TIME',
                               title=plot_title,
                               y_label='Seconds',
                               out_filename=plot_filename)
#                               y_limit=450)

            #### Deliberation time
            plot_title = 'Deliberation Time, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'deliberation-{0}-{1}.pdf'.format(task_file, start_config)
            plot_overall_stat( af = af,
                               attr_name='DELIBERATION_TIME',
                               title=plot_title,
                               y_label='Seconds',
                               out_filename=plot_filename)
#                               y_limit=9)

            #### Execution time
            plot_title = 'Execution Time, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'execution-{0}-{1}.pdf'.format(task_file, start_config)
            plot_overall_stat( af = af,
                               attr_name='EXECUTION_PHASE_TIME',
                               title=plot_title,
                               y_label='Seconds',
                               out_filename=plot_filename)
#                               y_limit=450)

            #### Execution time
            plot_title = 'Movement Time, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'movement-{0}-{1}.pdf'.format(task_file, start_config)
            plot_overall_stat( af = af,
                               attr_name='TOTAL_MOVEMENT_TIME',
                               title=plot_title,
                               y_label='Seconds',
                               out_filename=plot_filename)
#                               y_limit=450)

            #### Waiting time
            plot_title = 'Waiting Time, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'waiting-{0}-{1}.pdf'.format(task_file, start_config)
            plot_overall_stat( af = af,
                               attr_name='TOTAL_WAITING_TIME',
                               title=plot_title,
                               y_label='Seconds',
                               out_filename=plot_filename)
#                               y_limit=450)

            #### Idle time
            plot_title = 'Idle Time, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'idle-{0}-{1}.pdf'.format(task_file, start_config)
            plot_overall_stat( af = af,
                               attr_name='TOTAL_IDLE_TIME',
                               title=plot_title,
                               y_label='Seconds',
                               out_filename=plot_filename)
#                               y_limit=400)

            #### Delay time
            plot_title = 'Overall Delay Time, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'delay-{0}-{1}.pdf'.format(task_file, start_config)
            plot_overall_stat( af = af,
                               attr_name='TOTAL_DELAY_TIME',
                               title=plot_title,
                               y_label='Seconds',
                               out_filename=plot_filename )

            # #### Distance Travelled
            # plot_title = 'Overall Distance Travelled, "{0}", {1} start'.format(point_config, start_config)
            # plot_filename = 'distance-{0}-{1}.pdf'.format(point_config, start_config)
            # plot_overall_stat( experiments=exps,
            #                    attr_name='total_travel_distance',
            #                    title=plot_title,
            #                    y_label='cm',
            #                    out_filename=plot_filename )

            #### Distance Travelled
            plot_title = 'Total Distance Travelled, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'distance-{0}-{1}.pdf'.format(task_file, start_config)
            plot_overall_stat( af = af,
                               attr_name='TOTAL_DISTANCE',
                               title=plot_title,
                               y_label='meters',
                               out_filename=plot_filename)
#                               y_limit=50)

            #### Near Collisions
            plot_title = 'Overall Near Collisions, "{0}", {1} start'.format(task_file, start_config)
            plot_filename = 'collisions-{0}-{1}.pdf'.format(task_file, start_config)
            plot_overall_stat( af = af,
                               attr_name='TOTAL_COLLISIONS',
                               title=plot_title,
                               y_label='',
                               out_filename=plot_filename )

            # #### Distance travelled per robot
            # plot_title = 'Distance per Robot: "{0}", {1} start'.format(task_file, start_config)
            # plot_filename = 'distance-per-robot-{0}-{1}.pdf'.format(task_file, start_config)
            # plot_per_robot_stat( af = af,
            #                      attr_name='TOTAL_DISTANCE',
            #                      title=plot_title,
            #                      y_label='cm',
            #                      out_filename=plot_filename )
            
            # #### Travel time per robot
            # plot_title = 'Travel time per Robot: "{0}", {1} start'.format(task_file, start_config)
            # plot_filename = 'travel-time-per-robot-{0}-{1}.pdf'.format(task_file, start_config)
            # plot_per_robot_stat( af = af,
            #                      attr_name='robot_travel_time',
            #                      title=plot_title,
            #                      y_label='Seconds',
            #                      out_filename=plot_filename )
            
            # #### Idle time per robot
            # plot_title = 'Idle time per Robot: "{0}", {1} start'.format(task_file, start_config)
            # plot_filename = 'idle-time-per-robot-{0}-{1}.pdf'.format(task_file, start_config)
            # plot_per_robot_stat( af = af,
            #                      attr_name='robot_idle_time',
            #                      title=plot_title,
            #                      y_label='Seconds',
            #                      out_filename=plot_filename )

            # #### Delay time per robot
            # plot_title = 'Delay time per Robot: "{0}", {1} start'.format(task_file, start_config)
            # plot_filename = 'delay-per-robot-{0}-{1}.pdf'.format(task_file, start_config)
            # plot_per_robot_stat( af = af,
            #                      attr_name='total_delay_time',
            #                      title=plot_title,
            #                      y_label='Seconds',
            #                      out_filename=plot_filename )

if __name__ == '__main__':
    main(sys.argv[1:])
