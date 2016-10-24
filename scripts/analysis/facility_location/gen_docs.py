#!/usr/bin/env python
"""gen_docs.py

This is a meta-script that invokes other (plotting) scripts and makes use of a LaTeX template
to produce a document (PDF) containing plots of performance metrics, trajectories, and timelines
for a given experimental configuration.

"""


# Python libraries
import argparse
import glob
import os
import os.path
import pprint
import shutil
import subprocess


# Debugging
pp = pprint.PrettyPrinter(indent=4)

DEFAULT_HOSTNAME = 'eschneider'
DEFAULT_SCRIPT_DIR = os.path.join(os.environ['HOME'], 'GIT/mrta/scripts/analysis/facility_location')
DEFAULT_TEMPLATE_DIR = os.path.join(os.environ['HOME'], 'GIT/mrta/docs/stat_templates/facility_location')

plot_stats_script = 'plot_stats.py'
trajectory_script = 'plot_trajectory.py'
gen_trajectory_script = 'gen_trajectory_latex.py'
timeline_script = 'plot_timeline.py'
gen_timeline_script = 'gen_timeline_latex.py'

template_filename = 'stat_summary.tex'
legend_filename = 'timeline_legend.pdf'


def gen_docs(title, out_filename, script_dir, template_dir):
    # pp.pprint(task_file)
    # pp.pprint(bag_paths)
    # pp.pprint(hostname)
    # pp.pprint(script_dir)
    # pp.pprint(template_dir)

    # task_file_base = task_file.replace('.yaml', '')
    # task_file_base = task_file_base.replace('.txt', '')

    # 1. Run plot_stats.py to plot statistics for the given task file
    plot_path = os.path.join(script_dir, plot_stats_script)
    print("plot_path: {0}".format(plot_path))
    subprocess.check_call([plot_path, 'stats.csv', '--oldstarts'], shell=False)

    # # 2. Run plot_trajectory.py to plot trajectories
    # trajectory_path = os.path.join(script_dir, trajectory_script)
    # subprocess.check_call([trajectory_path] + bag_paths, shell=False)
    #
    # # 3. Run plot_timeline.py to plot timelines
    # timeline_path = os.path.join(script_dir, timeline_script)
    # print("timeline_path: {0}".format(timeline_path))
    # subprocess.check_call([timeline_path] + bag_paths, shell=False)

    # 4. Copy the stat_summary.tex tempalte to the current directory. Change its name
    #    to reflect the given hostname and task file.
    # document_filename = '{0}.tex'.format(task_file_base)
    document_filename = '{0}.tex'.format(out_filename)

    shutil.copy(os.path.join(template_dir, template_filename), document_filename)

    # 5. Replace the $TASKFILE$ and $HOSTNAME$ tokens in the template.

    # Read in the file
    with open(document_filename, 'r') as f:
        filedata = f.read()

    # Replace the target string
    # filedata = filedata.replace('$TASKFILE$', task_file_base)

    filedata = filedata.replace('$TITLE$', title)

    # Write the file out again
    with open(document_filename, 'w') as f:
        f.write(filedata)

    # 6. Generate trajectory LaTeX source (included by the main document).
    # gen_trajectory_path = os.path.join(script_dir, gen_trajectory_script)
    # traj_include_file = open('{0}-trajectories.tex'.format(task_file_base), 'wb')
    # subprocess.check_call([gen_trajectory_path, task_file_base], stdout=traj_include_file, shell=False)
    # traj_include_file.close()
    #
    # # 7. Generate timeline LaTeX source (included by the main document).
    # gen_timeline_path = os.path.join(script_dir, gen_timeline_script)
    # timeline_include_file = open('{0}-timelines.tex'.format(task_file_base), 'wb')
    # subprocess.check_call([gen_timeline_path, task_file_base], stdout=timeline_include_file, shell=False)
    # timeline_include_file.close()
    #
    # shutil.copy(os.path.join(template_dir, legend_filename), '.')

    # 8. Run pdflatex
    subprocess.check_call(['pdflatex', document_filename], shell=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate a PDF document with summary statistics, trajectories, and timelines for a given experimental configuration.')

    # parser.add_argument('-t', '--task_file',
    #                     type=str,
    #                     required=True,
    #                     help='Task file to document.')

    # parser.add_argument('--hostname',
    #                     type=str,
    #                     default=DEFAULT_HOSTNAME,
    #                     help='Name of the host on which experiments were run.')
    parser.add_argument('title',
                        help='Title to appear at the top of the document')

    parser.add_argument('out_filename',
                        help='Name of the LaTeX/pdf file to output')

    parser.add_argument('--script_dir',
                        type=str,
                        default=DEFAULT_SCRIPT_DIR,
                        help='Location of analysis scripts (e.g., parse_stats.py).')

    parser.add_argument('--template_dir',
                        type=str,
                        default=DEFAULT_TEMPLATE_DIR,
                        help='Location of document templates.')

#    parser.add_argument('--')

    # parser.add_argument('bag_file',
    #                     nargs='*',
    #                     help='Path to the bag file of the experiment to plot.')

    args = parser.parse_args()

    title = args.title
    out_filename = args.out_filename

    # bag_files = args.bag_file
    # task_file = args.task_file
    # hostname = args.hostname
    script_dir = args.script_dir
    template_dir = args.template_dir

    # # Make one, flat list of paths to log files
    # bag_paths = []
    #
    # for path_arg in bag_files:
    #     bag_paths.extend(glob.glob(path_arg))

    gen_docs(title, out_filename, script_dir, template_dir)
