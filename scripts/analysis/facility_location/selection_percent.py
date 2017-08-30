#!/usr/bin/env python

# Python libraries
import argparse
from collections import defaultdict
import math
import sys

# Stats/plotting libraries
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pprint
from scipy import stats

pp = pprint.PrettyPrinter(indent=4)

# Some global constants
mechanisms = ['SSI', 'PSI', 'SEL']
start_configs = ['clustered', 'distributed']

# task_files = ['SR-IT-DA-scenario2.yaml', 'SR-CT-DA-scenario2.yaml',
#               'MR-IT-DA-scenario2.yaml', 'MR-CT-DA-scenario2.yaml']


def main(stats_files, oldstarts):

    results = pd.read_csv(stats_file)

    if oldstarts:
        for start_config in start_configs:

            print start_config

            sel_frame = results[results.MECHANISM=='SEL'][results.START_CONFIG==start_config]

            sel_psi_count = len(sel_frame[sel_frame.MECHANISM_SELECTED=='PSI'])
            sel_ssi_count = len(sel_frame[sel_frame.MECHANISM_SELECTED=='SSI'])

            print "psi count: {}".format(sel_psi_count)
            print "ssi count: {}".format(sel_ssi_count)

            total_count = sel_psi_count + sel_ssi_count

            print "PSI selection % {0}".format(1. * sel_psi_count / total_count)
            print "SSI selection % {0}".format(1. * sel_ssi_count / total_count)

    print "combined"

    sel_frame = results[results.MECHANISM == 'SEL']

    sel_psi_count = len(sel_frame[sel_frame.MECHANISM_SELECTED == 'PSI'])
    sel_ssi_count = len(sel_frame[sel_frame.MECHANISM_SELECTED == 'SSI'])

    print "psi count: {}".format(sel_psi_count)
    print "ssi count: {}".format(sel_ssi_count)

    total_count = sel_psi_count + sel_ssi_count

    print "PSI selection % {0}".format(1. * sel_psi_count / total_count)
    print "SSI selection % {0}".format(1. * sel_ssi_count / total_count)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Plot statistics for MRTeAm experiment results.')
    parser.add_argument('stats_file', help='Path to a CSV file containing raw data for metrics.')

    parser.add_argument('--oldstarts', dest='oldstarts', action='store_true',
                        help='If set, also make separate plots for clustered and distributed starts.')
    # parser.add_argument('--no-oldstarts', dest='oldstarts', action='store_false')
    parser.set_defaults(oldstarts=False)

    args = parser.parse_args()

    stats_file = args.stats_file
    oldstarts = args.oldstarts

    main(stats_file, oldstarts)
