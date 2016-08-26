#!/usr/bin/env python

import argparse
from collections import defaultdict
import csv
import glob
import math
import numpy as np
import os
import pandas as pd
import pprint
import re
import rosbag
import sys

import mrta
import mrta.msg

IN_CSV_FILENAME = 'stats.csv'
OUT_DIST_CSV_FILENAME = 'median_distance.csv'
OUT_TIME_CSV_FILENAME = 'median_time.csv'

OUT_FIELDNAMES = ['TOTAL_MEDIAN_DISTANCE',
                  'MEDIAN_SPREAD',
                  'ROBOT1_MEDIAN_DISTANCE',
                  'ROBOT1_STARTX',
                  'ROBOT1_STARTY',
                  'ROBOT2_MEDIAN_DISTANCE',
                  'ROBOT2_STARTX',
                  'ROBOT2_STARTY',
                  'ROBOT3_MEDIAN_DISTANCE',
                  'ROBOT3_STARTX',
                  'ROBOT3_STARTY',
                  'MECHANISM']


pp = pprint.PrettyPrinter(indent=4)


def write_training_files(in_file, out_dist, out_time):

    try:

        stats_frame = pd.read_csv(in_file)

        out_dist_csv = csv.writer(open(out_dist, 'wb'))
        out_dist_csv.writerow(OUT_FIELDNAMES)

        out_time_csv = csv.writer(open(out_time, 'wb'))
        out_time_csv.writerow(OUT_FIELDNAMES)

        for name, group in stats_frame.groupby([stats_frame.ROBOT1_STARTX, stats_frame.ROBOT1_STARTY]):

            # Only record a training instance if runs for all three mechanisms (PSI, SSI, OSI) succeeded
            if group.ROBOT1_STARTX.count() < 3:
                continue

            # find the rows of the "winners" for both distance and time
            min_dist_idx = group.TOTAL_DISTANCE.idxmin()
            min_dist_row = stats_frame.ix[min_dist_idx]
            min_time_idx = group.TOTAL_MOVEMENT_TIME.idxmin()
            min_time_row = stats_frame.ix[min_time_idx]

            print "distance winner: {0}, time winner: {1}".format(min_dist_row.MECHANISM, min_time_row.MECHANISM)

            robot_median_distances = [min_dist_row.ROBOT1_MEDIAN_DISTANCE,
                                      min_dist_row.ROBOT2_MEDIAN_DISTANCE,
                                      min_dist_row.ROBOT3_MEDIAN_DISTANCE]

            print "robot median distances: {0}".format(robot_median_distances)

            min_median_dist = min(robot_median_distances)
            print "min_median_dist: {0}".format(min_median_dist)

            max_median_dist = max(robot_median_distances)
            print "max_median_dist: {0}".format(max_median_dist)


            median_spread = max_median_dist - min_median_dist
            print "median distance spread: {0}".format(median_spread)

            min_dist_row['MEDIAN_SPREAD'] = median_spread
            min_time_row['MEDIAN_SPREAD'] = median_spread

            out_dist_csv.writerow([min_dist_row[f] for f in OUT_FIELDNAMES])
            out_time_csv.writerow([min_time_row[f] for f in OUT_FIELDNAMES])

    except IOError:
        print "Something went wrong!"


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Produce training files from summary statistics.')


    parser.add_argument('-i', '--input',
                        default = IN_CSV_FILENAME,
                        help = 'The .csv file containing summary statistics,')

    parser.add_argument('-od', '--out_distance',
                        default = OUT_DIST_CSV_FILENAME,
                        help = 'The file to write for minimum distance-based training.')

    parser.add_argument('-ot', '--out_time',
                        default = OUT_TIME_CSV_FILENAME,
                        help = 'The file to write for minimum time-based training.')

    args = parser.parse_args()

    in_file = args.input
    out_dist = args.out_distance
    out_time = args.out_time

    print(in_file, out_dist, out_time)

    write_training_files(in_file, out_dist, out_time)
