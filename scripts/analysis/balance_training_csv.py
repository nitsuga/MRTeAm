#!/usr/bin/env python

import argparse
import csv
import os.path
import pandas as pd
import sys

from sklearn import preprocessing

from imblearn.combine import SMOTEENN
from imblearn.under_sampling import ClusterCentroids
from imblearn.under_sampling import RandomUnderSampler
from imblearn.under_sampling import TomekLinks
from imblearn.ensemble import EasyEnsemble


TRAINING_FILES = ['execution_phase_time.csv',
                  'minimax_distance.csv',
                  'run_time.csv',
                  'team_distance.csv']


def balance_training(in_dir, out_dir, method, significant=False):

    for t_file in TRAINING_FILES:

        in_t_filepath = os.path.join(in_dir, t_file)
        out_t_filepath = os.path.join(out_dir, t_file)

        print("Reading from {0}".format(in_t_filepath))
        df = pd.read_csv(in_t_filepath)

        if significant:
            df = df[df.WINNER_DIFFERENCE > df.WINNER_DIFFERENCE.std()]

        # Encode labels as integers
        le = preprocessing.LabelEncoder()

        # Drop the TASK_FILE column
        df.drop(['TASK_FILE'], axis=1, inplace=True)

        df_x = df.drop(['MECHANISM'], axis=1)
    #    df_y = df.ix[:, -1:].values.flatten().tolist()
        df_y = df.ix[:, 'MECHANISM'].values.flatten().tolist()

        # print("df_y: {0}".format(df_y))
        #
        # le.fit(df_y)
        # df_y_encoded = le.transform(df_y)
        #
        # print("df_y_encoded: {0}".format(df_y_encoded))

        # Resampler
        rs = None

        if method == 'UR':
            # Random undersampling
            rs = RandomUnderSampler()
        elif method == 'SE':
            # SMOTE + ENN
            rs = SMOTEENN()
        elif method == 'TL':
            rs = TomekLinks()
        elif method == 'CC':
            rs = ClusterCentroids()
        elif method == 'EE':
            rs = EasyEnsemble()
        else:
            print "Unsupported method: {0}".format(method)
            sys.exit(1)

        x_resampled, y_resampled = rs.fit_sample(df_x, df_y)
        # x_resampled, y_resampled = rs.fit_sample(df_x, df_y_encoded)

        x_resampled = x_resampled.tolist()
        y_resampled = y_resampled.tolist()
        # y_resampled = le.inverse_transform(y_resampled).tolist()

        with open(out_t_filepath, 'wb') as csvfile:
            print("Writing to {0}".format(out_t_filepath))
            writer = csv.writer(csvfile)
            # writer.writerow(OUT_FIELDNAMES)
            writer.writerow(df.columns.tolist())
            for i in range(len(x_resampled)):
                # print "writing row {0}".format(i)
                row = list(x_resampled[i])
                row.append(y_resampled[i])
                writer.writerow(row)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Balance training files.')

    parser.add_argument('in_dir',
                        help='The directory containing csv training files to balance.')
    parser.add_argument('out_dir',
                        help='The destination directory for balanced training files.')

    parser.add_argument('method',
                        choices=['UR', 'SE', 'TL', 'CC', 'EE'],
                        help='Method to balance training sets. UR: Undersample-random, SE: SMOTE-ENN, TL: Tomek-Links, CC: Cluster Centroids, EE: Easy Ensemble')

    parser.add_argument("-s", "--significant", help="Drop instances where WINNER_DISTANCE is not significant.",
                        action="store_true")

    args = parser.parse_args()
    in_dir = args.in_dir
    out_dir = args.out_dir
    method = args.method
    significant = args.significant

    balance_training(in_dir, out_dir, method)
