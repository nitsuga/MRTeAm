#!/usr/bin/env python

from collections import defaultdict
import glob
import sys


def main():
    clustered = {'RR': defaultdict(list),
                 'OSI': defaultdict(list),
                 'SSI': defaultdict(list),
                 'PSI': defaultdict(list),
                 'SUM': defaultdict(list),
                 'MAX': defaultdict(list)}
    distributed = {'RR': defaultdict(list),
                   'OSI': defaultdict(list),
                   'SSI': defaultdict(list),
                   'PSI': defaultdict(list),
                   'SUM': defaultdict(list),
                   'MAX': defaultdict(list)}

    glob_token = sys.argv[1]

    for traj_file in glob.glob('trajectories/*{0}*.png'.format(glob_token)):
        file_tokens = traj_file.split('_')
        (prefix, map, start_config, mechanism, task_file, remainder) = traj_file.split('__')

        if start_config == 'clustered':
            clustered[mechanism][task_file].append(traj_file)
        elif start_config == 'distributed':
            distributed[mechanism][task_file].append(traj_file)

    print "\\subsection*{Clustered}"
    
    for mechanism in clustered.keys():
        task_dict = clustered[mechanism]

        print "\\subsubsection*{{{0}}}".format(mechanism)

        for task_file in sorted(task_dict.keys()):

            # print "\\subsubsection*{{{0}}}".format(task_file)

            file_list = task_dict[task_file]

            for filename in sorted(file_list):
#                 print """\\begin{{minipage}}{{0.33\\textwidth}}
# \\centering
# {0}
# \\includegraphics[width=\\textwidth]{{{1}}}
# \\vspace{{0.4cm}}
# \\end{{minipage}}""".format(filename.replace('_','\_').replace('.png',''), filename)

                print """\\begin{{minipage}}{{0.33\\textwidth}}
\\centering
\\includegraphics[width=\\textwidth]{{{0}}}
\\vspace{{0.4cm}}
\\end{{minipage}}""".format(filename)

    print "\\section*{Distributed}"
    
    for mechanism in distributed.keys():
        task_dict = distributed[mechanism]

        print "\\subsection*{{{0}}}".format(mechanism)

        for task_file in sorted(task_dict.keys()):

            # print "\\subsubsection*{{{0}}}".format(task_file)

            file_list = task_dict[task_file]

            for filename in sorted(file_list):
#                 print """\\begin{{minipage}}{{0.33\\textwidth}}
# \\centering
# {0}
# \\includegraphics[width=\\textwidth]{{{1}}}
# \\vspace{{0.4cm}}
# \\end{{minipage}} """.format(filename.replace('_','\_').replace('.png',''), filename)
                print """\\begin{{minipage}}{{0.33\\textwidth}}
\\centering
\\includegraphics[width=\\textwidth]{{{0}}}
\\vspace{{0.4cm}}
\\end{{minipage}} """.format(filename)

if __name__ == '__main__':
    main()
