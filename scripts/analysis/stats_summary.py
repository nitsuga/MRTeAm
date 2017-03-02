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
                      'MAXIMUM_ROBOT_DISTANCE_MEAN', 'MAXIMUM_ROBOT_DISTANCE_STD']

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

            out_csv.writerow(out_row)

        for name, group in stats_frame.groupby(['MECHANISM']):
            out_row = []

            out_row.extend(['COMBINED', name])
            out_row.append(group.EXECUTION_PHASE_TIME.mean())
            out_row.append(group.EXECUTION_PHASE_TIME.std())
            out_row.append(group.MAXIMUM_ROBOT_DISTANCE.mean())
            out_row.append(group.MAXIMUM_ROBOT_DISTANCE.std())

            out_csv.writerow(out_row)

        out_file.close()


        out_file.close()

    except:
        print(sys.exc_info()[0])
        return -1

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
