include "hokuyo.inc"

define turtlebot position
(
  size [0.35 0.35 0.45]
  origin [0 0 0 0]
  gui_nose 1
  drive "diff"
  hokuyo(pose [ 0.0 0.000 0 0.000 ])
  bitmap "tb_top_plate.png"

  obstacle_return 1
  ranger_return 1.0
  laser_return 1.0
)
