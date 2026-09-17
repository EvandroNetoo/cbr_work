uint8 RED=1
uint8 BLUE=2

# Color, pose and quality belong to the same detected external contour.
uint8 color
float64 contour_area_px
# contour area / area of its fitted minimum-area rectangle.
float64 rectangularity
# RMS reprojection error of the known external rectangle, in pixels.
float64 pose_error
geometry_msgs/Pose pose
