#!/usr/bin/env python

# Python libraries
import argparse
from collections import defaultdict
import math
import sys

# Stats/plotting libraries
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pprint
from scipy import stats

pp = pprint.PrettyPrinter(indent=4)

# Some global constants
mechanisms = ['SSI', 'PSI', 'SEL']
start_configs = ['clustered', 'distributed']

# task_files = ['SR-IT-DA-scenario2.yaml', 'SR-CT-DA-scenario2.yaml',
#               'MR-IT-DA-scenario2.yaml', 'MR-CT-DA-scenario2.yaml']

robot_names = ['robot_1',
               'robot_2',
               'robot_3']

colors = ([0.0, 1.0, 0.0],  # 'green'
          [1.0, 0.0, 0.0],  # 'red'
          [0.0, 0.0, 1.0],  # 'blue'
          [0.5, 0.0, 1.0])  # 'violet'


class Experiment(object):
    def __init__(self, bag=None, mechanism=None, start_config=None, task_file=None):
        self.bag = bag
        self.mechanism = mechanism
        self.start_config = start_config
        self.task_file = task_file


def plot_overall_stat(af=None, attr_name=None, title=None, y_label=None, out_filename=None, y_limit=None, plot_samples=False):
    print "Plotting {0}".format(out_filename)

    # A dict of <mechanism_name> => <list of values>
    # stat_by_mechanism = defaultdict(list)

    # Populate said lists of values, by mechanism
    # for exp in experiments:
    #     # If exp.<attr_name> is an attribute, collect its value. If it's an
    #     # instance method, call it to obtain its value
    #     attr_value = getattr(exp, attr_name)
    #     if attr_value.__func__:
    #         stat_by_mechanism[exp.mechanism].append(attr_value.__call__())
    #     else:
    #         stat_by_mechanism[exp.mechanism].append(attr_value)

    #plt.title(title)
    #plt.ylabel(y_label)
    stat_means = []
    stat_errors = []

    for i, mechanism in enumerate(mechanisms):
        sample = af[af.MECHANISM == mechanism][attr_name]
        sample_mean = np.mean(sample)
        sample_error = stats.sem(sample)

        if math.isnan(sample_mean):
            sample_mean = 0.0

        if math.isnan(sample_error):
            sample_error = 0.0

        # print("sample_mean: {0}".format(pp.pformat(sample_mean)))

        stat_means.append(sample_mean)
        stat_errors.append(sample_error)

        if plot_samples:
            # Plot sample values as red dots
            plt.plot([(i+.15) for x in sample], sample, 'r.')

    # x-coordinates of the bars
    x_pos = np.arange(len(mechanisms))
    # Plot bars with std errors
    plt.bar(x_pos, stat_means, yerr=stat_errors, capsize=8,
            color=[0.75, 0.75, 0.75], width=0.6, align='center')

    # Bar labels
    # plt.tick_params(axis='x', which='major', labelsize=18)
    plt.tick_params(axis='x', which='major', labelsize=26)
    plt.tick_params(axis='y', which='major', labelsize=20)

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

    #plt.title(title)
    #plt.ylabel(y_label)
    stat1_means = []
    stat2_means = []
    stat1_errors = []
    stat2_errors = []

    for i, mechanism in enumerate(mechanisms):
        sample1 = af[af.MECHANISM == mechanism][attr1_name]

        sample1_mean = np.mean(sample1)
        sample1_error = stats.sem(sample1)

        if math.isnan(sample1_mean):
            sample1_mean = 0.0

        if math.isnan(sample1_error):
            sample1_error = 0.0

        stat1_means.append(sample1_mean)
        stat1_errors.append(sample1_error)

        sample2 = af[af.MECHANISM == mechanism][attr2_name]

        sample2_mean = np.mean(sample2)
        sample2_error = stats.sem(sample2)

        if math.isnan(sample2_mean):
            sample2_mean = 0.0

        if math.isnan(sample2_error):
            sample2_error = 0.0

        stat2_means.append(sample2_mean)
        stat2_errors.append(sample2_error)

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
    #plt.tick_params(axis='x', which='major', labelsize=18)
    plt.tick_params(axis='x', which='major', labelsize=26)
    plt.tick_params(axis='y', which='major', labelsize=20)

    plt.xticks(x_pos, mechanisms)

    ymin, ymax = plt.ylim()

    if y_limit:
        plt.ylim(0, y_limit)
    else:
        plt.ylim(0, ymax)

    plt.tight_layout()

    plt.savefig(out_filename)
    plt.close()


def plot_per_robot_stat(af=None, attr_name=None, title=None, y_label=None, out_filename=None, y_limit=None):
    print "Plotting {0}".format(out_filename)

    bar_width = 0.25
    x_start = 0.25
    x_pos = np.arange(x_start, len(mechanisms) + x_start)
    plt.title(title)
    plt.ylabel(y_label)

    stat_means = defaultdict(list)
    stat_errors = defaultdict(list)

    for i, mechanism in enumerate(mechanisms):
        for j, robot_name in enumerate(robot_names):
            # print "{0}, {1}".format(mechanism, robot_name)
            sample = af[af.MECHANISM == mechanism]['{0}_{1}'.format(robot_name.replace('_', '').upper(), attr_name)]
            stat_means[robot_name].append(np.mean(sample))
            stat_errors[robot_name].append(stats.sem(sample))

    bars = []
    # Plot bars with std errors
    for i, robot_name in enumerate(robot_names):
        grey_level = 0.25 + (i*0.25)
        bars.append(plt.bar([x + (bar_width*i) for x in x_pos],
                    stat_means[robot_name], yerr=stat_errors[robot_name],
                    color=[grey_level] * 3, width=bar_width))

    # Bar labels
    plt.xticks([x+0.33 for x in x_pos], mechanisms)    
    ymin, ymax = plt.ylim()

    # plt.ylim(0, ymax+(ymax*0.2))
    if y_limit:
        plt.ylim(0, y_limit)
    else:
        plt.ylim(0, ymax+(ymax*0.2))

    plt.xlim(0, x_pos[-1] + 1)
    plt.legend([bar[0] for bar in bars], robot_names)

    plt.savefig(out_filename)
    plt.close()


def plot_stacked_per_robot_stat(af=None, attr_names=[], title=None, y_label=None, out_filename=None, y_limit=None):
    print "Plotting {0}".format(out_filename)

    bar_width = 0.25
    x_start = 0.25
    x_pos = np.arange(x_start, len(mechanisms) + x_start)
    plt.title(title)
    plt.ylabel(y_label)

    stat_means_by_attr = defaultdict(lambda: defaultdict(list))
    stat_errors_by_attr = defaultdict(lambda: defaultdict(list))

    for mechanism in mechanisms:
        for robot_name in robot_names:
            for attr_name in attr_names:
                # print "{0}, {1}, {2}".format(mechanism, robot_name, attr_name)
                sample = af[af.MECHANISM==mechanism]['{0}_{1}'.format(robot_name.replace('_','').upper(), attr_name)]
                # pp.pprint(sample)
                stat_means_by_attr[robot_name][attr_name].append(np.mean(sample))
                stat_errors_by_attr[robot_name][attr_name].append(stats.sem(sample))

    bars = []
    # Plot bars with std errors

    for i, robot_name in enumerate(robot_names):

        r_level = 0.25 + (i*0.25)
        g_level = 0.25 + (i*0.25)
        b_level = 0.25 + (i*0.25)

        # grey_level = 0.5 + (i*0.25)
        grey_level = 0.75

        # last_means = None
        bottoms = [0.0 for x in mechanisms]
        for j, attr_name in enumerate(attr_names):
            # print("{0}-{1}".format(robot_name, attr_name))
            # print("bottoms: {0}".format(pp.pformat(bottoms)))

            stat_means = stat_means_by_attr[robot_name][attr_name]
            stat_errors = stat_errors_by_attr[robot_name][attr_name]

            # print("means: {0}".format(pp.pformat(stat_means)))

            color = colors[j]

            bars.append(plt.bar([x + (bar_width*i) for x in x_pos],
                        stat_means_by_attr[robot_name][attr_name],
                        color=[x * grey_level for x in color], width=bar_width, bottom=bottoms))

            bottoms = map(lambda x, y: x+y, bottoms, stat_means)
            # last_means = stat_means_by_attr[robot_name][attr_name]

    # Bar labels
    plt.xticks([x+0.33 for x in x_pos], mechanisms)
    ymin, ymax = plt.ylim()

    # plt.ylim(0, ymax+(ymax*0.2))
    if y_limit:
        plt.ylim(0, y_limit)
    else:
        plt.ylim(0, ymax+(ymax*0.2))

    plt.xlim(0, x_pos[-1] + 1)
    # plt.legend( [bar[0] for bar in bars], robot_names )
    # plt.legend(bars, attr_names, loc='upper center', bbox_to_anchor=(0.5, -0.25), ncol=2, fancybox=True)
    plt.legend(bars, attr_names, loc='upper center', ncol=2, fancybox=True, shadow=True)

    plt.savefig(out_filename)
    plt.close()


def main(stats_file, oldstarts):

    aframe = pd.read_csv(stats_file)

    af = aframe

    # Style?
    # plt.style.use('bmh')
    # plt.style.use('dark_background')
    # plt.style.use('ggplot')
    # plt.style.use('ggplot')
    # plt.style.use('grayscale')


    #### Execution phase time
    plot_title = 'Execution Phase Time'
    plot_filename = 'execution-phase-time.pdf'
    plot_overall_stat(af=af,
                      attr_name='EXECUTION_PHASE_TIME',
                      title=plot_title,
                      y_label='Seconds',
                      out_filename=plot_filename,
                      y_limit=600
                      )

    #### Deliberation time
    plot_title = 'Deliberation Time'
    plot_filename = 'deliberation-time.pdf'
    plot_overall_stat(af=af,
                      attr_name='DELIBERATION_TIME',
                      title=plot_title,
                      y_label='Seconds',
                      out_filename=plot_filename,
                      #y_limit=60
                      )

    #### Total run time
    plot_title = 'Total Run Time'
    plot_filename = 'run-time.pdf'
    plot_overall_stat(af=af,
                      attr_name='TOTAL_RUN_TIME',
                      title=plot_title,
                      y_label='Seconds',
                      out_filename=plot_filename,
                      y_limit=600
                      )
    # y_limit=300)

    # Total run time stacked: deliberation time + execution time
    # Total run time
    plot_title = 'Stacked Run Time'
    plot_filename = 'stacked-runtime.pdf'
    plot_stacked_stats(af=af,
                       attr1_name='DELIBERATION_TIME',
                       attr2_name='EXECUTION_PHASE_TIME',
                       title=plot_title,
                       y_label='Seconds',
                       out_filename=plot_filename,
                       y_limit=600
                       )

    #### Team distance travelled
    plot_title = 'Team Distance'
    plot_filename = 'team-distance.pdf'
    plot_overall_stat(af=af,
                      attr_name='TOTAL_DISTANCE',
                      title=plot_title,
                      y_label='meters',
                      out_filename=plot_filename)

    #### Maximum robot distance travelled
    plot_title = 'Maximum Robot Distance'
    plot_filename = 'maximum-distance.pdf'
    plot_overall_stat(af=af,
                      attr_name='MAXIMUM_ROBOT_DISTANCE',
                      title=plot_title,
                      y_label='meters',
                      out_filename=plot_filename)

    # Stats by start config
    if oldstarts:
        for start_config in start_configs:

            af = aframe[aframe.START_CONFIG == start_config]

            # Execution phase time
            plot_title = 'Execution Phase Time, {0} start'.format(start_config)
            plot_filename = 'execution-phase-time-{0}.pdf'.format(start_config)
            plot_overall_stat(af = af,
                              attr_name='EXECUTION_PHASE_TIME',
                              title=plot_title,
                              y_label='Seconds',
                              out_filename=plot_filename,
                              y_limit=600
                              )

            # Deliberation time
            plot_title = 'Deliberation Time, {0} start'.format(start_config)
            plot_filename = 'deliberation-time-{0}.pdf'.format(start_config)
            plot_overall_stat(af = af,
                              attr_name='DELIBERATION_TIME',
                              title=plot_title,
                              y_label='Seconds',
                              out_filename=plot_filename,
                              y_limit=60
                              )

            # Total run time
            plot_title = 'Total Run Time, {0} start'.format(start_config)
            plot_filename = 'run-time-{0}.pdf'.format(start_config)
            plot_overall_stat(af = af,
                              attr_name='TOTAL_RUN_TIME',
                              title=plot_title,
                              y_label='Seconds',
                              out_filename=plot_filename,
                              y_limit=600)

            # Total run time stacked: deliberation time + execution time
            # Total run time
            plot_title = 'Stacked Run Time, {0} start'.format(start_config)
            plot_filename = 'stacked-runtime-{0}.pdf'.format(start_config)
            plot_stacked_stats(af=af,
                               attr1_name='DELIBERATION_TIME',
                               attr2_name='EXECUTION_PHASE_TIME',
                               title=plot_title,
                               y_label='Seconds',
                               out_filename=plot_filename,
                               y_limit=600
                               )

            # Team distance travelled
            plot_title = 'Team Distance, {0} start'.format(start_config)
            plot_filename = 'team-distance-{0}.pdf'.format(start_config)
            plot_overall_stat(af = af,
                              attr_name='TOTAL_DISTANCE',
                              title=plot_title,
                              y_label='meters',
                              out_filename=plot_filename)
                              # y_limit=80)

            # Maximum distance travelled
            plot_title = 'Maximum Robot Distance, {0} start'.format(start_config)
            plot_filename = 'maximum-distance-{0}.pdf'.format(start_config)
            plot_overall_stat(af = af,
                              attr_name='MAXIMUM_ROBOT_DISTANCE',
                              title=plot_title,
                              y_label='meters',
                              out_filename=plot_filename)
                              # y_limit=80)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Plot statistics for MRTeAm experiment results.')
    parser.add_argument('stats_file', help='Path to a CSV file containing raw data for metrics.')

    parser.add_argument('--oldstarts', dest='oldstarts', action='store_true',
                        help='If set, also make separate plots for clustered and distributed starts.')
    # parser.add_argument('--no-oldstarts', dest='oldstarts', action='store_false')
    parser.set_defaults(oldstarts=False)

    args = parser.parse_args()

    stats_file = args.stats_file
    oldstarts = args.oldstarts


    main(stats_file, oldstarts)
