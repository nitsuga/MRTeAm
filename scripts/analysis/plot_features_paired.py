#!/usr/bin/env python


import argparse
from itertools import product
import os
import os.path
import string
import sys

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

X_features = ['TOTAL_DISTANCE_TO_ASSIGNED_MEDIANS',
              'TOTAL_DISTANCE_TO_ALL_MEDIANS',
              'MAX_DISTANCE_TO_ASSIGNED_MEDIAN',
              'MAX_DISTANCE_TO_ANY_MEDIAN',
              'MIN_DISTANCE_TO_ASSIGNED_MEDIAN',
              'MIN_DISTANCE_TO_ANY_MEDIAN',
              'ASSIGNED_MEDIAN_DISTANCE_SPREAD',
              'TOTAL_MEDIAN_DISTANCE_SPREAD',
              'TEAM_DIAMETER',
              'GREEDY_MEDIAN_COUNT_SPREAD',
              'MECHANISM']

Y_metrics = ['MAXIMUM_ROBOT_DISTANCE',
             'TOTAL_DISTANCE',
             'EXECUTION_PHASE_TIME',
             'TOTAL_RUN_TIME']


plot_types = ['scatter']

def main(training_dir):

    # Set some seaborn style defaults
    sns.set(style='darkgrid')

    dir_entries = os.listdir(training_dir)

    for dir_entry in dir_entries:

        file_path = os.path.join(training_dir, dir_entry)

        if os.path.isfile(file_path) and os.path.splitext(file_path)[1] == '.csv':
            plot_features(file_path)


def plot_features(input_csv):

    input_basename = os.path.splitext(input_csv)[0]

    # print "file basename == {0}".format(input_basename)

    print 'Reading {0}...'.format(input_csv)

    try:
        stats = pd.read_csv(input_csv)

        stats = stats.ix[:, X_features]

        pairgrid = sns.pairplot(stats, hue='MECHANISM', hue_order=['PSI','','SSI'], palette='bright', size=4)

        out_filename = '{0}.pdf'.format(input_basename)
        print 'Writing {0}'.format(out_filename)
        plt.savefig(out_filename)

        plt.close(pairgrid.fig)


        # for pair in product(X_features, Y_metrics):
        #
        #     X = pair[0]
        #     Y = pair[1]
        #
        #     for plot_type in plot_types:
        #
        #         if not os.path.exists(plot_type):
        #             os.makedirs(plot_type)
        #
        #         print "Plotting {0} by {1}".format(X, Y)
        #
        #         # Plot PSI
        #         ax = stats[stats.MECHANISM=='PSI'].plot(kind=plot_type, x=X, y=Y, alpha=0.5, color='blue', label='PSI')
        #
        #         # Plot SSI on the sames axes
        #         stats[stats.MECHANISM == 'SSI'].plot(kind=plot_type, x=X, y=Y, alpha=0.5, color='red', label='SSI', ax=ax)
        #
        #         # Make axis labels more readable
        #         x_label = ' '.join(string.capwords(X, '_').split('_'))
        #         y_label = ' '.join(string.capwords(Y, '_').split('_'))
        #         x_label = x_label.replace('To ', 'to ')
        #         y_label = y_label.replace('To ', 'to ')
        #
        #         if plot_type == 'hist':
        #             y_label = 'Count'
        #
        #         plt.xlabel(x_label)
        #         plt.ylabel(y_label)
        #
        #         out_filename = '{0}_{1}_by_{2}.pdf'.format(plot_type, X, Y)
        #         out_path = os.path.join(plot_type, out_filename)
        #         print 'Writing {0}'.format(out_path)
        #         plt.savefig(out_path)
        #
        #
        #         plt.close(ax.get_figure())

    except:
        print sys.exc_info()
        sys.exit(1)



if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Plot')

    parser.add_argument('training_dir',
                        help='Directory containing training CSV files.')

    args = parser.parse_args()
    training_dir = args.training_dir

    main(training_dir)