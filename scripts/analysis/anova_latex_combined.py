#!/usr/bin/env python

import os, os.path
import re
import sys
import yaml


def get_script_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))

# Degrees of freedom
dof1, dof2 = sys.argv[1:3]

sim_dir = 'simulation'
phys_dir = 'physical'

metrics_config_file = os.path.join(get_script_path(), 'anova_latex.yaml')
output_dir = 'formatted'

table_header = """
%{{\centering
%\\begin{{table}}[h]
%{{\small
%\\begin{{tabularx}}{{\\textwidth}}{{|XrX|XrX||XrX|XrX|}}
%\multicolumn{{6}}{{|X||}}{{Physical}}   & \multicolumn{{6}}{{X|}}{{Simulation}} \\\\
\multicolumn{{12}}{{c}}{{(a) {2}}} \\\\
\hline
 & $F(3,{0})$ & $p$ & & $F(3,{0})$ & $p$ & & $F(3,{1})$ & $p$ & & $F(3,{1})$ & $p$ \\\\
\hline
"""

table_body = """
\multicolumn{{3}}{{|l|}}{{MR-CT-DA-}}   & \multicolumn{{3}}{{l||}}{{SR-CT-DA-}} & \multicolumn{{3}}{{l|}}{{MR-CT-DA-}}   & \multicolumn{{3}}{{l|}}{{SR-CT-DA-}} \\\\
cl-s1 & ${0}$	    & ${1}$     & cl-s1	& ${16}$	& ${17}$    & cl-s1 & ${32}$    & ${33}$    & cl-s1	& ${48}$	& ${49}$ \\\\
di-s1 & ${2}$	    & ${3}$     & di-s1	& ${18}$	& ${19}$    & di-s1 & ${34}$    & ${35}$    & di-s1	& ${50}$	& ${51}$ \\\\
cl-s2 & ${4}$	    & ${5}$     & cl-s2	& ${20}$	& ${21}$    & cl-s2 & ${36}$    & ${37}$    & cl-s2	& ${52}$	& ${53}$ \\\\
di-s2 & ${6}$	    & ${7}$     & di-s2	& ${22}$	& ${23}$    & di-s2 & ${38}$    & ${39}$    & di-s2	& ${54}$	& ${55}$ \\\\
\hline
\multicolumn{{3}}{{|l|}}{{MR-IT-DA-}}   & \multicolumn{{3}}{{l||}}{{SR-IT-DA-}} & \multicolumn{{3}}{{l|}}{{MR-IT-DA-}}   & \multicolumn{{3}}{{l|}}{{SR-IT-DA-}} \\\\
cl-s1	& ${8}$	    & ${9}$     & cl-s1	& ${24}$	& ${25}$    & cl-s1	& ${40}$	& ${41}$    & cl-s1	& ${56}$	& ${57}$ \\\\
di-s1	& ${10}$	& ${11}$    & di-s1	& ${26}$	& ${27}$    & di-s1	& ${42}$	& ${43}$    & di-s1	& ${58}$	& ${59}$ \\\\
cl-s2	& ${12}$	& ${13}$    & cl-s2	& ${28}$	& ${29}$    & cl-s2	& ${44}$	& ${45}$    & cl-s2	& ${60}$	& ${61}$ \\\\
di-s2	& ${14}$	& ${15}$    & di-s2	& ${30}$	& ${31}$    & di-s2	& ${46}$	& ${47}$    & di-s2	& ${62}$	& ${63}$ \\\\
\hline
"""

table_footer = """
\end{{tabularx}}
}}
\caption{{{0}}}
\label{{{1}}}
\end{{table}}
}}
"""

anova_pattern = re.compile("F\(\d+,\d+\)=(\d+\.\d+)\tp<(0.\d\d\d)$")

metrics_file = open(metrics_config_file, 'rb')
metrics = yaml.load(metrics_file)

for metric in metrics:
    metric_name = metric['name']
    metric_caption = metric['caption']
    metric_label = metric['label']

    values = []
    input_filename = "{0}-f.txt".format(metric_name)

    for subdir in phys_dir, sim_dir:
        input_path = os.path.join(subdir, input_filename)
        print "opening {0}".format(input_path)
        input_file = open(input_path, 'rb')

        for line in input_file:
            # print line
            match = re.search(anova_pattern, line)

            if not match:
                print("Can't find ANOVA values in input! Aborting.")
                sys.exit(1)

            f_ratio = match.group(1)
            p_value = match.group(2)
            if p_value == '0.950':
                p_value = '\\textbf{0.950}'

            # values.extend([match.group(1), match.group(2)])
            values.extend([f_ratio, p_value])

    print "len(values): {0}".format(len(values))
    print "values: {0}".format(values)

    # output_text = table_header.format(dof1, dof2) + table_body.format(*values) + table_footer.format(metric_caption, metric_label)
    output_text = table_header.format(dof1, dof2, metric_caption) + table_body.format(*values)

    out_file = open(os.path.join(output_dir, "{0}-f.txt".format(metric_name)), 'wb')
    out_file.write(output_text)
    out_file.close()
