define hokuyo ranger
(
  sensor(
    range [0 4.0]
    fov 270.25
    samples 481
  )
  color "black"
  size [ 0.05 0.05 0.1 ]
#  ctrl "lasernoise"
)


define lds01 ranger
(
  sensor(
    range [0 3.5]
    fov 360
    samples 1800
  )
  color "black"
  size [ 0.0695 0.0955 0.0395 ]
#  ctrl "lasernoise"
)
