include "hokuyo.inc"

define turtlebot position
(
#  size [0.35 0.35 0.45]
  size [0.3 0.3 0.45]
  origin [0 0 0 0]
  gui_nose 1
  drive "diff"
  hokuyo(pose [ 0.0 0.0 0.4 0.000 ])
  bitmap "tb_top_plate.png"
#  boundary 1

  obstacle_return 1
  ranger_return 1.0
  laser_return 1.0

  stack_children 0
)
