#!/usr/bin/env python

from collections import defaultdict
import glob
import sys


def main():
    clustered = {'RR': defaultdict(list),
                 'OSI': defaultdict(list),
                 'SSI': defaultdict(list),
                 'PSI': defaultdict(list)}
    distributed = {'RR': defaultdict(list),
                   'OSI': defaultdict(list),
                   'SSI': defaultdict(list),
                   'PSI': defaultdict(list)}

    glob_token = sys.argv[1]

    for timeline_file in glob.glob('timelines/*{0}*.pdf'.format(glob_token)):
        # print("Reading {0}".format(timeline_file))
        (prefix, map, start_config, mechanism, task_file, remainder) = timeline_file.split('__')

        if start_config == 'clustered':
            clustered[mechanism][task_file].append(timeline_file)
        elif start_config == 'distributed':
            distributed[mechanism][task_file].append(timeline_file)

    print "\\subsection*{Clustered}"
    
    for mechanism in clustered.keys():
        task_dict = clustered[mechanism]

        print "\\subsubsection*{{{0}}}".format(mechanism)

        for task_file in sorted(task_dict.keys()):

            # print "\\subsubsection*{{{0}}}".format(task_file)

            file_list = task_dict[task_file]

#            print "\\begin{center}"

            for filename in sorted(file_list):
                print """\\includegraphics[width=\\textwidth]{{{0}}}
\\\\[1cm]""".format(filename)

#            print "\\end{center}"

    print "\\section*{Distributed}"
    
    for mechanism in distributed.keys():
        task_dict = distributed[mechanism]

        print "\\subsection*{{{0}}}".format(mechanism)

        for task_file in sorted(task_dict.keys()):

            # print "\\subsubsection*{{{0}}}".format(task_file)

            file_list = task_dict[task_file]

#            print "\\begin{center}"

            for filename in sorted(file_list):
                print """\\includegraphics[width=\\textwidth]{{{0}}}
\\\\[1cm]""".format(filename)

#            print "\\end{center}"

if __name__ == '__main__':
    main()
