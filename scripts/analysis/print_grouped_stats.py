#!/usr/bin/env python

import argparse
import numpy as np
import pandas as pd
import scipy as sp
import scipy.stats as st
import sys

MECHANISMS = ['PSI', 'SSI', 'SEL']
METRICS = ['EXECUTION_PHASE_TIME', 'TOTAL_RUN_TIME', 'MAXIMUM_ROBOT_DISTANCE', 'TOTAL_DISTANCE']


# From http://stackoverflow.com/questions/15033511/compute-a-confidence-interval-from-sample-data
def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), st.sem(a)
    h = se * st.t._ppf((1+confidence)/2., n-1)
    return m, h, m-h, m+h


def print_grouped_stats(stats_csv):
    try:
        stats = pd.read_csv(stats_csv)
    except:
        print("Couldn't open/parse {0}! Exiting.".format(stats_csv))
        sys.exit(1)

    group_stats = stats[stats.MECHANISM.isin(MECHANISMS)]

    # 'Successful' group stats, i.e. a start+task configuration for which runs
    # with all mechanisms in MECHANISMS succeeded
    mech_group_stats = pd.DataFrame()

    # for name, group in group_stats.groupby('TASK_FILE'):
    for name, group in group_stats.groupby('SCENARIO_ID'):
        # print name
        # print len(group)
        if len(group) == len(MECHANISMS):
            mech_group_stats = mech_group_stats.append([group.iloc[idx] for idx in range(len(MECHANISMS))])

    print("{0} groups of {1}".format(len(mech_group_stats)/len(MECHANISMS), len(MECHANISMS)))

    # mech_start_group_stats = pd.DataFrame()
    # for name, group in group_stats.groupby(['TASK_FILE', 'START_CONFIG']):
    #     # print name
    #     # print len(group)
    #     if len(group) == len(MECHANISMS):
    #         mech_start_group_stats = mech_start_group_stats.append([group.iloc[idx] for idx in range(len(MECHANISMS))])
    #
    # print("{0} groups of {1}".format(len(mech_start_group_stats)/len(MECHANISMS), len(MECHANISMS)))

    for mechanism in MECHANISMS:

        print(mechanism)

        for metric in METRICS:
            data = mech_group_stats[mech_group_stats.MECHANISM==mechanism][metric]
            mean, ci, low, high = mean_confidence_interval(data)

            print("  {0}".format(metric))
            # print("   mean: {0}".format(mech_group_stats[mech_group_stats.MECHANISM==mechanism][metric].mean()))
            # print("    std: {0}".format(mech_group_stats[mech_group_stats.MECHANISM==mechanism][metric].std()))
            print("   mean: {0} +- {1} ({2}, {3})".format(mean, ci, low, high))
            # print("    std: {0}".format(mech_group_stats[mech_group_stats.MECHANISM==mechanism][metric].std()))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Print comparative stats (mean, std.) for basic metrics.')

    parser.add_argument('stats_csv',
                        help='CSV file containing experiment statistics.')
    args = parser.parse_args()

    stats_csv = args.stats_csv
    print_grouped_stats(stats_csv)
