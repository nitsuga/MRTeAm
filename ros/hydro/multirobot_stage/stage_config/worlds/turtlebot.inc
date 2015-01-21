include "hokuyo.inc"

define turtlebot position
(
  size [0.35 0.35 0.45]
  origin [0 0 0 0]
  gui_nose 1
  drive "diff"
  hokuyo(pose [ 0.0 0.000 0 0.000 ])
  bitmap "tb-top-plate2.png"

  obstacle_return 1
  ranger_return 1
  laser_return 1
)
