#!/usr/bin/env python

import csv
import pandas as pd
import sys

FIELD_NAMES = [
    'BAG_FILENAME',
    'DATETIME',
    'MAP',
    'START_CONFIG',
    'MECHANISM',
    'TASK_FILE',
    'TOTAL_RUN_TIME',
    'DELIBERATION_TIME',
    'EXECUTION_PHASE_TIME',
    'NAP_TIME',
    'TOTAL_MOVEMENT_TIME',
    'TOTAL_EXECUTION_TIME',
    'TOTAL_WAITING_TIME',
    'TOTAL_IDLE_TIME',
    'TOTAL_DELAY_TIME',
    'TOTAL_DISTANCE',
    'TOTAL_COLLISIONS',
    'TOTAL_DISTANCE_TO_ASSIGNED_MEDIANS',
    'MEDIAN_TASK_IDS',
    'NUM_ANNOUNCE_MSGS',
    'NUM_ANNOUNCE_TASKS',
    'NUM_BID_MSGS',
    'ALLOC_MSG_BYTES',
    'ROBOT1_STARTX',
    'ROBOT1_STARTY',
    'ROBOT1_DISTANCE_TO_ASSIGNED_MEDIAN',
    'ROBOT1_DISTANCE',
    'ROBOT1_MOVEMENT_TIME',
    'ROBOT1_WAITING_TIME',
    'ROBOT1_IDLE_TIME',
    'ROBOT1_DELAY_TIME',
    'ROBOT2_STARTX',
    'ROBOT2_STARTY',
    'ROBOT2_DISTANCE_TO_ASSIGNED_MEDIAN',
    'ROBOT2_DISTANCE',
    'ROBOT2_MOVEMENT_TIME',
    'ROBOT2_WAITING_TIME',
    'ROBOT2_IDLE_TIME',
    'ROBOT2_DELAY_TIME',
    'ROBOT3_STARTX',
    'ROBOT3_STARTY',
    'ROBOT3_DISTANCE_TO_ASSIGNED_MEDIAN',
    'ROBOT3_DISTANCE',
    'ROBOT3_MOVEMENT_TIME',
    'ROBOT3_WAITING_TIME',
    'ROBOT3_IDLE_TIME',
    'ROBOT3_DELAY_TIME',
    'MEAN_MSG_TIME',
    'MAXIMUM_ROBOT_DISTANCE',
    'MECHANISM_SELECTED',
    'MECHANISM_SELECTION_TIME'
]


def main(argv):

    instats = csv.DictReader(open('stats.csv', 'rb'))

    out_stats = csv.writer(open('stats-corrected.csv', 'wb'))
    out_stats.writerow(FIELD_NAMES)

    for row in instats:
        mst = float(row['MECHANISM_SELECTION_TIME'])
        # print mst
        if row['MECHANISM'] == 'SSI' or row['MECHANISM'] == 'PSI':
            # print('Mechanism: {0}'.format(row['MECHANISM']))
            # print('Old deliberation time: {0}'.format(row['DELIBERATION_TIME']))
            # print('Old run time: {0}'.format(row['TOTAL_RUN_TIME']))

            row['DELIBERATION_TIME'] = float(row['DELIBERATION_TIME']) - mst
            row['TOTAL_RUN_TIME'] = float(row['TOTAL_RUN_TIME']) - mst

            # print('New deliberation time: {0}'.format(row['DELIBERATION_TIME']))
            # print('New run time: {0}'.format(row['TOTAL_RUN_TIME']))

        row_values = [row[f] for f in FIELD_NAMES]
        # print row_values
        out_stats.writerow(row_values)


if __name__ == '__main__':
    main(sys.argv)
