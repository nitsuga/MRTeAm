#!/usr/bin/env python

import os, os.path
import re
import sys
import yaml


def get_script_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))

metrics_config_file = os.path.join(get_script_path(), 'anova_latex.yaml')
output_dir = 'formatted'

table_template = """
{{\centering
\\begin{{table}}[h]
{{\small
\\begin{{tabular}}{{|lrr||lrr|}}
 & $F(3,8)$ & $p$ & & $F(3,8)$ & $p$ \\\\
\hline
\multicolumn{{3}}{{|l||}}{{MR-CT-DA-}}   & \multicolumn{{3}}{{|l|}}{{SR-CT-DA-}} \\\\
cl-s1 & ${2}$	    & ${3}$     & cl-s1	& ${18}$	& ${19}$ \\\\
di-s1 & ${4}$	    & ${5}$     & di-s1	& ${20}$	& ${21}$ \\\\
cl-s2 & ${6}$	    & ${7}$     & cl-s2	& ${22}$	& ${23}$ \\\\
di-s2 & ${8}$	    & ${9}$     & di-s2	& ${24}$	& ${25}$ \\\\
\hline
\multicolumn{{3}}{{|l||}}{{MR-IT-DA-}}   & \multicolumn{{3}}{{|l|}}{{SR-IT-DA-}} \\\\
cl-s1	& ${10}$	& ${11}$    & cl-s1	& ${26}$	& ${27}$ \\\\
di-s1	& ${12}$	& ${13}$    & di-s1	& ${28}$	& ${29}$ \\\\
cl-s2	& ${14}$	& ${15}$    & cl-s2	& ${30}$	& ${31}$ \\\\
di-s2	& ${16}$	& ${17}$    & di-s2	& ${32}$	& ${33}$ \\\\
\hline
\end{{tabular}}
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

    input_file = open("{0}-f.txt".format(metric_name), 'rb')

    values = []
    for line in input_file:
        # print line
        match = re.search(anova_pattern, line)

        if not match:
            print("Can't find ANOVA values in input! Aborting.")
            sys.exit(1)

        values.extend([match.group(1), match.group(2)])

    # print "values: {0}".format(values)
    output_text = table_template.format(metric_caption, metric_label, *values)

    out_file = open(os.path.join(output_dir, "{0}-f.txt".format(metric_name)), 'wb')
    out_file.write(output_text)
    out_file.close()
