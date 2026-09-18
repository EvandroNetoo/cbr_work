uint8 RED=1
uint8 BLUE=2

# Color, pose and quality belong to the same detected external contour.
uint8 color
float64 contour_area_px
# contour area / area of its fitted minimum-area rectangle.
float64 rectangularity
# Full view: PnP reprojection RMS px. Partial view: visible-edge fit RMS px.
float64 pose_error
float64 external_width_m
float64 external_depth_m
# Physical height above the work surface; independent of detected pose.z.
float64 external_height_m
uint32 observation_count
float64 position_spread_m
float64 yaw_spread_deg
bool partial
float64 position_uncertainty_m
float64 yaw_uncertainty_deg
float64 partial_fit_overlap
geometry_msgs/Pose pose
