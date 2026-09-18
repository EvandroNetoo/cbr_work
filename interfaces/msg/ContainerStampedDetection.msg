std_msgs/Header header
uint8 RED=1
uint8 BLUE=2
uint8 color
float64 contour_area_px
float64 rectangularity
# Full view: PnP reprojection RMS px. Partial view: visible-edge fit RMS px.
float64 pose_error
float64 external_width_m
float64 external_depth_m
# Physical height above the work surface; independent of detected pose.z.
float64 external_height_m
# Temporal support used by the action result. Per-frame publications use 1/0.
uint32 observation_count
float64 position_spread_m
float64 yaw_spread_deg
# True when the external silhouette intersects the image boundary.
bool partial
# Additional XY/yaw uncertainty of a fitted partial silhouette.
float64 position_uncertainty_m
float64 yaw_uncertainty_deg
# Intersection-over-union of the fitted and visible external silhouettes.
float64 partial_fit_overlap
geometry_msgs/Pose pose
