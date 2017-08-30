#!/usr/bin/env python

import argparse
from collections import defaultdict
import csv
import glob
import math
import numpy as np
import os
import pandas as pd
import re
import sys


DEFAULT_CSV_INPUT = 'stats.csv'
DEFAULT_CSV_OUTPUT = 'stats_summary.csv'


def stats_summary(input_filename, output_filename):
    try:

        stats_frame = pd.read_csv(input_filename)

        out_fields = ['START_CONFIG', 'MECHANISM',
                      'EXECUTION_PHASE_TIME_MEAN', 'EXECUTION_PHASE_TIME_STD',
                      'MAXIMUM_ROBOT_DISTANCE_MEAN', 'MAXIMUM_ROBOT_DISTANCE_STD',
                      'DELIBERATION_TIME_MEAN', 'DELIBERATION_TIME_STD',
                      'TOTAL_RUN_TIME_MEAN', 'TOTAL_RUN_TIME_STD']

        out_file = open(output_filename, 'wb')

        out_csv = csv.writer(out_file)

        out_csv.writerow(out_fields)

        for name, group in stats_frame.groupby(['START_CONFIG', 'MECHANISM']):
            out_row = []

            out_row.extend(name)
            out_row.append(group.EXECUTION_PHASE_TIME.mean())
            out_row.append(group.EXECUTION_PHASE_TIME.std())
            out_row.append(group.MAXIMUM_ROBOT_DISTANCE.mean())
            out_row.append(group.MAXIMUM_ROBOT_DISTANCE.std())
            out_row.append(group.DELIBERATION_TIME.mean())
            out_row.append(group.DELIBERATION_TIME.std())
            out_row.append(group.TOTAL_RUN_TIME.mean())
            out_row.append(group.TOTAL_RUN_TIME.std())

            out_csv.writerow(out_row)

        for name, group in stats_frame.groupby(['MECHANISM']):
            out_row = []

            out_row.extend(['COMBINED', name])
            out_row.append(group.EXECUTION_PHASE_TIME.mean())
            out_row.append(group.EXECUTION_PHASE_TIME.std())
            out_row.append(group.MAXIMUM_ROBOT_DISTANCE.mean())
            out_row.append(group.MAXIMUM_ROBOT_DISTANCE.std())
            out_row.append(group.DELIBERATION_TIME.mean())
            out_row.append(group.DELIBERATION_TIME.std())
            out_row.append(group.TOTAL_RUN_TIME.mean())
            out_row.append(group.TOTAL_RUN_TIME.std())

            out_csv.writerow(out_row)

        out_file.close()

    except:
        print(sys.exc_info()[0])
        return -1

    out_best = open('stats_best.txt', 'wb')

    max_dist_psi = []
    max_dist_ssi = []
    exec_time_psi = []
    exec_time_ssi = []

    incsv = csv.DictReader(open(input_filename))

    for row in incsv:
        if row['MECHANISM'] == 'PSI':
            max_dist_psi.append(row['MAXIMUM_ROBOT_DISTANCE'])
            exec_time_psi.append(row['EXECUTION_PHASE_TIME'])
        elif row['MECHANISM'] == 'SSI':
            max_dist_ssi.append(row['MAXIMUM_ROBOT_DISTANCE'])
            exec_time_ssi.append(row['EXECUTION_PHASE_TIME'])

    print "{0}, {1}".format(exec_time_psi[0], exec_time_ssi[0])

    print "len(max_dist_psi) = {}".format(len(max_dist_psi))
    print "len(max_dist_ssi) = {}".format(len(max_dist_ssi))

    min_max_dist = []
    dist_psi_win_count = 0
    dist_ssi_win_count = 0
    for i, row in enumerate(max_dist_psi):
        minimax = 0

        if float(max_dist_psi[i]) < float(max_dist_ssi[i]):
            dist_psi_win_count += 1
            minimax = float(max_dist_psi[i])
        else:
            dist_ssi_win_count += 1
            minimax = float(max_dist_ssi[i])

        min_max_dist.append(minimax)

    out_best.write("{0}, {1}, {2}, {3}\n".format(np.mean(min_max_dist),
                                                 np.std(min_max_dist),
                                                 dist_psi_win_count,
                                                 dist_ssi_win_count))

    min_exec_time = []
    exec_time_psi_win_count = 0
    exec_time_ssi_win_count = 0

    for i, row in enumerate(exec_time_psi):
        mintime = 0

        if float(exec_time_psi[i]) < float(exec_time_ssi[i]):
            exec_time_psi_win_count += 1
            mintime = float(exec_time_psi[i])
        else:
            exec_time_ssi_win_count += 1
            mintime = float(exec_time_ssi[i])

        min_exec_time.append(mintime)

    out_best.write("{0}, {1}, {2}, {3}".format(np.mean(min_exec_time),
                                               np.std(min_exec_time),
                                               exec_time_psi_win_count,
                                               exec_time_ssi_win_count))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Given a stats file produced by parse_stats_csv.py, print summary statistics.')

    parser.add_argument('-i', '--input',
                        default=DEFAULT_CSV_INPUT,
                        help="Name of the input .csv file")

    parser.add_argument('-o', '--output',
                        default=DEFAULT_CSV_OUTPUT,
                        help="Name of the output .csv file")

    args = parser.parse_args()

    input_file = args.input
    output_file = args.output

    stats_summary(input_file, output_file)
