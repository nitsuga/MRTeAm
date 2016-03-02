#!/usr/bin/env python

import csv

import collections

import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pprint

import sys

pp = pprint.PrettyPrinter(indent=4)

CSV_IN_DEFAULT = 'stats.csv'

# ENVIRONMENTS = ['PHYS', 'SIM']
# IMS = ['FULL', 'MINIMAL']

MECHANISMS = ['RR', 'OSI', 'SSI', 'PSI']

START_CONFIGS = ['clustered', 'distributed']

# TASK_FILES = ['SR-IT-DA-scenario1.yaml',
#               'SR-CT-DA-scenario1.yaml',
#               'MR-IT-DA-scenario1.yaml',
#               'MR-CT-DA-scenario1.yaml',
#               'SR-IT-DA-scenario2.yaml',
#               'SR-CT-DA-scenario2.yaml',
#               'MR-IT-DA-scenario2.yaml',
#               'MR-CT-DA-scenario2.yaml']

TASK_FILES = ['MR-CT-DA-scenario1.yaml',
              'MR-CT-DA-scenario2.yaml',
              'MR-IT-DA-scenario1.yaml',
              'MR-IT-DA-scenario2.yaml',
              'SR-CT-DA-scenario1.yaml',
              'SR-CT-DA-scenario2.yaml',
              'SR-IT-DA-scenario1.yaml',
              'SR-IT-DA-scenario2.yaml']

METRICS = ['DELIBERATION_TIME',
           'EXECUTION_PHASE_TIME',
           'NAP_TIME',
           'TOTAL_COLLISIONS',
           'TOTAL_DELAY_TIME',
           'TOTAL_DISTANCE',
           'TOTAL_EXECUTION_TIME',
           'TOTAL_IDLE_TIME',
           'TOTAL_MOVEMENT_TIME',
           'TOTAL_RUN_TIME',
           'TOTAL_WAITING_TIME']

OUT_CSV_FILENAME = '{0}-rank.'


def rank_stats(input_csv):

    aframe = pd.read_csv(input_csv)

    if aframe.empty:
        print("Can't read data from {0}!".format(input_csv))
        sys.exit()

    # Interaction method rankings
    for metric in METRICS:
        print metric

        output_txt = csv.writer(open("{0}-rank.txt".format(metric), 'wb'), delimiter='\t')
        output_dat = csv.writer(open("{0}-rank.dat".format(metric), 'wb'), delimiter='\t')

        output_dat.writerow(['%'] + MECHANISMS + ['condition'])

        for task_file in TASK_FILES:
            for start_config in START_CONFIGS:

                stat_values = []

                condition = "{0}-{1}".format(task_file.replace('.yaml', ''), start_config)

                for mechanism in MECHANISMS:
                    stat_mean = np.mean(aframe[aframe.TASK_FILE==task_file][aframe.START_CONFIG==start_config][aframe.MECHANISM==mechanism][metric].values)
                    stat_values.append([mechanism, stat_mean])

                stat_values_sorted = sorted(stat_values, key=lambda x: x[1])
                stat_ranks = [x[0] for x in stat_values_sorted]

                print("im_values: {0}".format(pp.pformat(stat_values_sorted)))

                txt_row_data = [condition]
                dat_row_data = []

                for stat_value in stat_values:
                    mechanism, value = stat_value
                    rank = stat_ranks.index(mechanism) + 1
                    txt_row_data.append("({0},{1},{2})".format(mechanism, rank, value))

                    dat_row_data.append(rank)

                dat_row_data.append("% {0}".format(condition))

                print("txt_row_data: {0}".format(pp.pformat(txt_row_data)))
                print("dat_row_data: {0}".format(pp.pformat(dat_row_data)))

                output_txt.writerow(txt_row_data)
                output_dat.writerow(dat_row_data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Output rank orders for given summary statistics.')

    parser.add_argument("input_csv",
                        default=CSV_IN_DEFAULT,
                        help="CSV file containing input data")

    args = parser.parse_args()

    input_csv = args.input_csv

    rank_stats(input_csv)
