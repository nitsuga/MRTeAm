#!/usr/bin/env python

import argparse
import numpy as np
import pandas as pd
import scipy as sp
import scipy.stats as st
import sys

MECHANISMS = ['PSI', 'SSI']
METRICS = ['EXECUTION_PHASE_TIME',
           'TOTAL_RUN_TIME',
           'MAXIMUM_ROBOT_DISTANCE',
           'DELIBERATION_TIME',
           'MECHANISM_SELECTION_TIME']


metric_valid_threshold = 10000


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

    mission_id = 1000000
    for name, group in group_stats.groupby('SCENARIO_ID'):
        # print name
        # print len(group)

        group_valid = True

        if len(group) < len(MECHANISMS):
            print("At least one (mechanism) run for this mission failed, skipping...")
            group_valid = False
            continue

        rows = [group.iloc[idx] for idx in range(len(MECHANISMS))]

        for metric in METRICS:
            for row in rows:
                if row[metric] > metric_valid_threshold:
                    print("{0} has an unusually high value, skipping this mission...".format(metric))
                    group_valid = False
                    break

            if not group_valid:
                break

        if not group_valid:
            continue

        for row in rows:
            row['MISSION_ID'] = mission_id
        mech_group_stats = mech_group_stats.append(rows)
        mission_id += 1

    print("{0} groups of {1}".format(len(mech_group_stats)/len(MECHANISMS), len(MECHANISMS)))

    mech_group_stats.to_csv('paired_stats.csv', index=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Group statistics by mission (starting and task locations).')

    parser.add_argument('stats_csv',
                        help='CSV file containing experiment statistics.')
    args = parser.parse_args()

    stats_csv = args.stats_csv
    print_grouped_stats(stats_csv)
