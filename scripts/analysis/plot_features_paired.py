#!/usr/bin/env python


import argparse
from itertools import product, permutations
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


def main(training_dir, output_dir):

    if not os.path.exists(training_dir):
        print "{0} does not exist!".format(training_dir)
        sys.exit(1)

    if not os.path.exists(output_dir):
        print "{0} does not exist!".format(output_dir)
        sys.exit(1)

    # Set some seaborn style defaults
    sns.set(style='darkgrid')

    dir_entries = os.listdir(training_dir)

    for csv_file in dir_entries:

        file_path = os.path.join(training_dir, csv_file)

        if os.path.isfile(file_path) and os.path.splitext(file_path)[1] == '.csv':
            plot_features(training_dir, csv_file, output_dir)


def plot_features(training_dir, csv_file, output_dir):

    input_path = os.path.join(training_dir, csv_file)

    input_basename = os.path.splitext(csv_file)[0]

    # print "file basename == {0}".format(input_basename)

    print 'Reading {0}...'.format(input_path)

    try:
        stats = pd.read_csv(input_path)

        stats = stats.ix[:, X_features]

        pairgrid = sns.pairplot(stats, hue='MECHANISM', hue_order=['PSI', '', 'SSI'], palette='bright', size=4)

        pairgrid_filename = '{0}.pdf'.format(input_basename)
        pairgrid_path = os.path.join(output_dir, pairgrid_filename)
        print 'Writing {0}'.format(pairgrid_path)
        plt.savefig(pairgrid_path)

        plt.close(pairgrid.fig)

        # If none exists, create a directory for plots of individual elements of the pairgrid.
        # This directory is named after the input basename.
        plot_element_dir = os.path.join(output_dir, input_basename)

        if not os.path.exists(plot_element_dir):
            os.makedirs(plot_element_dir)

        for pair in permutations(X_features, 2):

            x_feature = pair[0]
            y_feature = pair[1]

            if x_feature == 'MECHANISM' or y_feature == 'MECHANISM':
                continue

            for plot_type in plot_types:

                print "Plotting {0} by {1}".format(x_feature, y_feature)

                # Plot PSI
                ax = stats[stats.MECHANISM=='PSI'].plot(kind=plot_type, x=x_feature, y=y_feature, alpha=0.5, color='blue', label='PSI')

                # Plot SSI on the sames axes
                stats[stats.MECHANISM == 'SSI'].plot(kind=plot_type, x=x_feature, y=y_feature, alpha=0.5, color='red', label='SSI', ax=ax)

                # Make axis labels more readable
                x_label = ' '.join(string.capwords(x_feature, '_').split('_'))
                y_label = ' '.join(string.capwords(y_feature, '_').split('_'))
                x_label = x_label.replace('To ', 'to ')
                y_label = y_label.replace('To ', 'to ')

                if plot_type == 'hist':
                    y_label = 'Count'

                plt.xlabel(x_label)
                plt.ylabel(y_label)

                plot_filename = '{0}_{1}_{2}_by_{3}.pdf'.format(input_basename, plot_type, x_feature, y_feature)
                plot_path = os.path.join(plot_element_dir, plot_filename)
                print 'Writing {0}'.format(plot_path)
                plt.savefig(plot_path)

                plt.close(ax.get_figure())

    except:
        print sys.exc_info()
        sys.exit(1)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Plot')

    parser.add_argument('training_dir',
                        help='Directory containing training CSV files.')

    parser.add_argument('output_dir',
                        help='Output directory for plot files.')

    args = parser.parse_args()
    training_dir = args.training_dir
    output_dir = args.output_dir

    main(training_dir, output_dir)
