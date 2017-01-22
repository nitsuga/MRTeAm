#!/usr/bin/env python

import argparse
import csv
import pandas as pd
import sys

from imblearn.combine import SMOTEENN
from imblearn.under_sampling import ClusterCentroids
from imblearn.under_sampling import RandomUnderSampler
from imblearn.under_sampling import TomekLinks


def balance_training(in_file, out_file, method, significant=False):
    df = pd.read_csv(in_file)

    if significant:
        df = df[df.WINNER_DIFFERENCE > df.WINNER_DIFFERENCE.std()]

    df_x = df.ix[:, :-1].values.tolist()
#    df_y = df.ix[:, -1:].values.flatten().tolist()
    df_y = df.ix[:, 'MECHANISM'].values.flatten().tolist()

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
    else:
        print "Unsupported method: {0}".format(method)
        sys.exit(1)

    x_resampled, y_resampled = rs.fit_sample(df_x, df_y)

    x_resampled = x_resampled.tolist()
    y_resampled = y_resampled.tolist()

    with open(out_file, 'wb') as csvfile:
        writer = csv.writer(csvfile)
        #writer.writerow(OUT_FIELDNAMES)
        writer.writerow(df.columns.tolist())
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

    parser.add_argument('method',
                        choices=['UR', 'SE', 'TL', 'CC'],
                        help='Method to balance training sets. UR: Undersample-random, SE: SMOTE-ENN')

    parser.add_argument("-s", "--significant", help="Drop instances where WINNER_DISTANCE is not significant.",
                        action="store_true")

    args = parser.parse_args()
    in_file = args.input
    out_file = args.output
    method = args.method
    significant = args.significant

    balance_training(in_file, out_file, method)
