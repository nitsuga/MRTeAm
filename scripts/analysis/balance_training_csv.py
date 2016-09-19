#!/usr/bin/env python

import argparse
import csv
import pandas as pd


from imblearn.under_sampling import RandomUnderSampler

OUT_FIELDNAMES = ['TOTAL_DISTANCE_TO_MEDIANS',
                  # 'TOTAL_DISTANCE_TO_ALL_MEDIANS',
                  'MEDIAN_SPREAD',
                  'MAX_DISTANCE_TO_MEDIAN',
                  'MIN_DISTANCE_TO_MEDIAN',
                  'TEAM_DIAMETER',
                  'MEDIAN_ASSIGNMENT_SPREAD',
                  'ROBOT1_DISTANCE_TO_MEDIAN',
                  # 'ROBOT1_DISTANCE_TO_ALL_MEDIANS',
                  'ROBOT1_STARTX',
                  'ROBOT1_STARTY',
                  'ROBOT2_DISTANCE_TO_MEDIAN',
                  # 'ROBOT2_DISTANCE_TO_ALL_MEDIANS',
                  'ROBOT2_STARTX',
                  'ROBOT2_STARTY',
                  'ROBOT3_DISTANCE_TO_MEDIAN',
                  # 'ROBOT3_DISTANCE_TO_ALL_MEDIANS',
                  'ROBOT3_STARTX',
                  'ROBOT3_STARTY',
                  'MECHANISM']


def balance_training(in_file, out_file):

    df = pd.read_csv(in_file)

    df_x = df.ix[:,:-1].values
    df_y = df.ix[:,-1:].values

    # Random undersampling
    rus = RandomUnderSampler()

    x_resampled, y_resampled = rus.fit_sample(df_x, df_y)

    x_resampled = x_resampled.tolist()
    y_resampled = y_resampled.tolist()

    with open(out_file, 'wb') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(OUT_FIELDNAMES)
        for i in range(len(x_resampled)):
            print "writing row {0}".format(i)
            row = list(x_resampled[i])
            row.append(y_resampled[i])
            writer.writerow(row)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Balance training files.')

    parser.add_argument('input',
                        help='The .csv training file to balance.')
    parser.add_argument('output',
                        help='The balanced csv file to write.')

    args = parser.parse_args()
    in_file = args.input
    out_file = args.output

    balance_training(in_file, out_file)
