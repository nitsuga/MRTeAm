#!/usr/bin/env python

import argparse
import pandas as pd
import sys

MECHANISMS = ['PSI', 'SSI', 'SEL']
METRICS = ['EXECUTION_PHASE_TIME', 'MAXIMUM_ROBOT_DISTANCE']


def print_grouped_stats(stats_csv):

    try:
        stats = pd.read_csv(stats_csv)
    except:
        print("Couldn't open/parse {0}! Exiting.".format(stats_csv))
        sys.exit(1)

    group_stats = stats[stats.MECHANISM.isin(MECHANISMS)]

    # 'Successful' group stats, i.e. a start+task configuration for which runs
    # with all mechanisms in MECHANISMS succeeded
    succ_group_stats = pd.DataFrame()

    for name, group in group_stats.groupby('TASK_FILE'):
        print name
        print len(group)
        if len(group) == len(MECHANISMS):
            succ_group_stats = succ_group_stats.append([group.iloc[idx] for idx in range(len(MECHANISMS))])

    for mechanism in MECHANISMS:

        print(mechanism)

        for metric in METRICS:
            print("  {0}".format(metric))
            print("   mean: {0}".format(succ_group_stats[succ_group_stats.MECHANISM==mechanism][metric].mean()))
            print("    std: {0}".format(succ_group_stats[succ_group_stats.MECHANISM==mechanism][metric].std()))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Print comparative stats (mean, std.) for basic metrics.')

    parser.add_argument('stats_csv',
                        help='CSV file containing experiment statistics.')
    args = parser.parse_args()

    stats_csv = args.stats_csv
    print_grouped_stats(stats_csv)
