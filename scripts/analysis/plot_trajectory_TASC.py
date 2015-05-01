#!/usr/bin/env python

import cairo
from collections import defaultdict
import glob
import math
import os
import re
import rosbag
import rospkg
import sys

IMG_WIDTH, IMG_HEIGHT = 602, 538
#IMG_WIDTH, IMG_HEIGHT = 800, 600

stroke_colors = { 'robot_1': (1.0, 0.0, 0.0),   # Red
                  'robot_2': (0.0, 1.0, 0.0),   # Green
                  'robot_3': (0.0, 0.0, 1.0)  } # Blue 

# Robot start locations
start_locations = { 'clustered': ( (100.0, 100.0),
                                   (50.0, 100.0),
                                   (50.0, 50.0) ),
                    'distributed': ( (552.0, 488.0),
                                     (50.0, 488.0),
                                     (50.0, 50.0) )

}

task_point_configs = {}

robot_names = [ 'robot_1',
                'robot_2',
                'robot_3' ]

def read_point_configs():
    rospack = rospkg.RosPack()

    task_file_dir = "{0}/task_files".format(rospack.get_path('mrta_auctioneer'))

    #for point_config in ['A', 'B', 'C', 'D', 'E']:
    #for task_file in ['brooklyn_tasks_A.txt', 'brooklyn_tasks_C.txt', 'tasks_A.txt']:
#    for task_file in ['TASC_scenario_6.txt']:
    for task_file in ['TASC_scenario_5.txt']:
        points = []
        config_file = open("{0}/{1}".format(task_file_dir, task_file), "rb")        
        for line in config_file:

            # Ignore comments
            if line.startswith('#'):
                continue

            # 'line' is in the format "s x y"
            # where s = seconds after start that the task appears
            # x,y = task location
            s, x, y = line.split()
            points.append([float(x)*100., float(y)*100.])
            
        task_point_configs[task_file] = points

def usage():
    print 'Usage: ' + sys.argv[0] + ': <paths_to_bag_file(s)>'

def draw_arena(ctx):
    """ Given a Cairo graphics context (ctx), draw the walls of the arena """
    # Arena line properties
    ctx.set_line_width( ( 3. / IMG_WIDTH ) )
    ctx.set_source_rgb(0, 0, 0)

    # Draw the outside of the arena
    ctx.rectangle(0, 0, 1, 1)
    ctx.stroke()

    ctx.set_source_rgb(0, 0, 0)

    # Set line end caps to square while drawing the walls
    default_line_cap = ctx.get_line_cap()
    ctx.set_line_cap(cairo.LINE_CAP_SQUARE)

    # Draw the walls
    ctx.move_to( 205. / IMG_WIDTH, 0 ) # w5
    ctx.line_to( 205. / IMG_WIDTH, 206. / IMG_HEIGHT )
    ctx.stroke()

    ctx.move_to( 105. / IMG_WIDTH, 206. / IMG_HEIGHT ) # w6
    ctx.line_to( 205. / IMG_WIDTH, 206. / IMG_HEIGHT )
    ctx.stroke()

    ctx.move_to( 102. / IMG_WIDTH, 336. / IMG_HEIGHT ) # w7
    ctx.line_to( 205. / IMG_WIDTH, 336. / IMG_HEIGHT )
    ctx.stroke()

    ctx.move_to( 205. / IMG_WIDTH, 336. / IMG_HEIGHT ) # w8
    ctx.line_to( 205. / IMG_WIDTH, 538. / IMG_HEIGHT )
    ctx.stroke()

    ctx.move_to( 422. / IMG_WIDTH, 336. / IMG_HEIGHT ) # w9
    ctx.line_to( 422. / IMG_WIDTH, 538. / IMG_HEIGHT )
    ctx.stroke()

    ctx.move_to( 315. / IMG_WIDTH, 336. / IMG_HEIGHT ) # w10
    ctx.line_to( 520. / IMG_WIDTH, 336. / IMG_HEIGHT )
    ctx.stroke()

    ctx.move_to( 315. / IMG_WIDTH, 206. / IMG_HEIGHT ) # w11
    ctx.line_to( 422. / IMG_WIDTH, 206. / IMG_HEIGHT )
    ctx.stroke()

    ctx.move_to( 422. / IMG_WIDTH, 0 ) # w12
    ctx.line_to( 422. / IMG_WIDTH, 206. / IMG_HEIGHT )
    ctx.stroke()

    # Restore the default line cap style
    ctx.set_line_cap(default_line_cap)

def main(argv):

    if len(argv) < 1:
        usage()
        sys.exit(1)

    read_point_configs()

    dt_re = re.compile('(.*)\.bag')

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in argv:
        bag_paths.extend(glob.glob(path_arg))

    for bag_path in bag_paths:
        print("Reading {0}".format(bag_path))

        bag = None
        try:
            bag = rosbag.Bag(bag_path)
        except:
            print("Couldn't open {0} for reading!".format(bag_path))
            continue

        bag_filename = os.path.basename(bag_path)
        (map, start_config, mechanism, task_file, remainder) = bag_filename.split('__')
        
        # Create a Cairo surface and get a handle to its context
        surface = cairo.ImageSurface (cairo.FORMAT_ARGB32, IMG_WIDTH, IMG_HEIGHT)
        ctx = cairo.Context (surface)

        # Invert the y-axis to make life easier
        ctx.transform( cairo.Matrix( yy = -1, y0 = IMG_HEIGHT ) )
    
        ctx.scale( IMG_WIDTH, IMG_HEIGHT)

        # Paint a white background
        ctx.set_source_rgb( 1.0, 1.0, 1.0 )
        ctx.rectangle( 0, 0, 1.0, 1.0 )
        ctx.fill()

        draw_arena(ctx)

        # Draw start locations
        for i, start_loc in enumerate(start_locations[start_config]):
            start_loc_x = start_loc[0]
            start_loc_y = start_loc[1]

            # E.g., "robot-1"
            stroke_color = stroke_colors["robot_%d" % (i+1)]

            ctx.set_source_rgb( stroke_color[0], stroke_color[1], stroke_color[2] )
            ctx.rectangle((start_loc_x-6) / IMG_WIDTH,
                          (start_loc_y-6) / IMG_HEIGHT,
                          12. / IMG_WIDTH, 12. / IMG_HEIGHT)
            ctx.stroke()

        # Reset pen to black
        ctx.set_source_rgb(0, 0, 0)
         
        # Draw task points
        task_points = task_point_configs[task_file]

        # Font style for printing points
        ctx.select_font_face('Sans', cairo.FONT_SLANT_NORMAL,
                             cairo.FONT_WEIGHT_BOLD)
        ctx.set_font_size(14. / IMG_WIDTH)

        ctx.set_line_width( ( 1. / IMG_WIDTH ) )

        for i, task_point in enumerate(task_points):
            task_x = task_point[0]
            task_y = task_point[1]

#            print "Drawing task point at (%f,%f)" % (task_x, task_y)

            # Draw point number (i) as a label slightly above and to the right of the point
            ctx.move_to( (task_x+6) / IMG_WIDTH, (task_y+6) / IMG_HEIGHT )
            ctx.transform( cairo.Matrix( yy = -1, y0 = IMG_HEIGHT ) )
            ctx.show_text(str(i+1))
            ctx.stroke()
            ctx.transform( cairo.Matrix( yy = -1, y0 = IMG_HEIGHT ) )

#            ctx.arc(task_x / IMG_WIDTH, task_y / IMG_HEIGHT, 5./IMG_WIDTH, 0, 2*math.pi)
#            ctx.stroke()
#            ctx.fill()
            ctx.move_to( (task_x-5) / IMG_WIDTH, (task_y-5) / IMG_HEIGHT )
            ctx.line_to( (task_x+5) / IMG_WIDTH, (task_y+5) / IMG_HEIGHT )
            ctx.stroke()

            ctx.move_to( (task_x-5) / IMG_WIDTH, (task_y+5) / IMG_HEIGHT )
            ctx.line_to( (task_x+5) / IMG_WIDTH, (task_y-5) / IMG_HEIGHT )
            ctx.stroke()

        # Trajectory line width
        ctx.set_line_width( ( 2. / IMG_WIDTH ) )
    
        run_msgs = defaultdict(list)

        for topic,msg,msg_time in bag.read_messages():
            run_msgs[topic].append(msg)

        for r_name in robot_names:

            # Stroke color
            stroke_color = stroke_colors[r_name]
            ctx.set_source_rgb( stroke_color[0], stroke_color[1], stroke_color[2] )
        
            start_pose = None
            for amcl_msg in run_msgs['/{0}/amcl_pose'.format(r_name)]:
                amcl_pose = amcl_msg.pose.pose
                pose_x = amcl_pose.position.x
                pose_y = amcl_pose.position.y

                if pose_x == 0 and pose_y == 0:
                    continue

                if start_pose is None:
                    ctx.move_to( pose_x * 100. / IMG_WIDTH, pose_y * 100. / IMG_HEIGHT )
                    start_pose = amcl_pose
                    #print "{0} start_pose: {1}:".format(r_name, start_pose)
                else:
                    ctx.line_to( pose_x * 100. / IMG_WIDTH, pose_y * 100. / IMG_HEIGHT)
                
            ctx.stroke()
            
        bag_basename = bag_filename.replace('.bag', '')
        surface.write_to_png( bag_basename + '.png' )

if __name__ == "__main__":
    main(sys.argv[1:])
