#!/usr/bin/env python


import argparse
from itertools import product
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
              'GREEDY_MEDIAN_COUNT_SPREAD']

Y_metrics = ['MAXIMUM_ROBOT_DISTANCE',
             'TOTAL_DISTANCE',
             'EXECUTION_PHASE_TIME',
             'TOTAL_RUN_TIME']

plot_types = ['scatter']


def main(input_csv):

    print 'Reading {0}...'.format(input_csv)

    try:
        stats = pd.read_csv(input_csv)

        for pair in product(X_features, Y_metrics):

            X = pair[0]
            Y = pair[1]

            for plot_type in plot_types:

                if not os.path.exists(plot_type):
                    os.makedirs(plot_type)

                print "Plotting {0} by {1}".format(X, Y)

                # Plot PSI
                ax = stats[stats.MECHANISM=='PSI'].plot(kind=plot_type, x=X, y=Y, alpha=0.5, color='blue', label='PSI')

                # Plot SSI on the sames axes
                stats[stats.MECHANISM == 'SSI'].plot(kind=plot_type, x=X, y=Y, alpha=0.5, color='red', label='SSI', ax=ax)

                # Make axis labels more readable
                x_label = ' '.join(string.capwords(X, '_').split('_'))
                y_label = ' '.join(string.capwords(Y, '_').split('_'))
                x_label = x_label.replace('To ', 'to ')
                y_label = y_label.replace('To ', 'to ')

                if plot_type == 'hist':
                    y_label = 'Count'

                plt.xlabel(x_label)
                plt.ylabel(y_label)

                out_filename = '{0}_{1}_by_{2}.pdf'.format(plot_type, X, Y)
                out_path = os.path.join(plot_type, out_filename)
                print 'Writing {0}'.format(out_path)
                plt.savefig(out_path)


                plt.close(ax.get_figure())

    except:
        print sys.exc_info()
        sys.exit(1)



if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Plot')

    parser.add_argument('input_csv',
                        help='The CSV file containing training statistics to plot.')

    args = parser.parse_args()
    input_csv = args.input_csv

    main(input_csv)