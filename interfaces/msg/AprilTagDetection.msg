# Identity and quality belong to the same detected pose.
string family
int32 id
float64 decision_margin
uint8 hamming
# RMS reprojection error of the selected IPPE pose, in pixels.
float64 pose_error
geometry_msgs/Pose pose
# Color of the cube carrying this tag; zero means the visible sides
# could not be classified. Pose quality is evaluated separately.
uint8 UNKNOWN=0
uint8 RED=1
uint8 BLUE=2
uint8 color
float32 color_confidence
uint32 color_observation_count
