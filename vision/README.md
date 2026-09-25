# Vision

`vision` serves `/vision/analyze_scene`. Each goal selects AprilTags (bit 1),
the white table surface (bit 4), HSV containers (bit 8), or a combination.
The camera is captured on demand, and each detector processes the newest
available rectified frame at its configured rate.

```bash
ros2 launch vision vision.launch.py
ros2 action send_goal /vision/analyze_scene interfaces/action/AnalyzeScene \
  "{requested_detectors: 8, duration: {sec: 2, nanosec: 0}, work_surface_height_m: 0.05}" --feedback
```

## HSV containers

The detector thresholds red and blue in HSV, cleans each mask with an OpenCV
open and close operation, and finds connected components. It applies separate
minimum pixel areas for complete and image-edge components, selected by work
surface height. A container is confirmed when its mask center remains within
`hsv_container_center_tolerance_px` for at least
`hsv_container_min_confirmed_frames` frames. The result contains color, mask
area, observation count, position spread, `partial`, and pose.

The pixel center is projected through the calibrated camera ray to a known
horizontal plane: work surface height plus `external_height_m` (0.073 m).
`floor_frame` supplies the floor reference, and TF expresses the result in
`base_frame`. For a cut container, the center is the center of its visible
mask. It may be offset from the center of the physical opening. Keep the arm
and camera still during the confirmation window.

The configured pixel areas depend on image resolution. Recalibrate the area
thresholds, border margin, and center tolerance when resolution changes.

## Debug images

`/containers/debug_image` shows the cleaned red and blue masks as translucent
colored regions with white boundaries. Live frames also show component centers
and areas. The retained final frame shows the masks from the last processed
container frame and the confirmed centers. `/apriltags/debug_image` shows tag
poses; `/table_surface/debug_image` shows usable and unknown table pixels.
A planned release target is projected over the stored container image when
`/manipulation/container_release_target` is published.

The parameters are in `config/vision.yaml`. The placement action uses the
returned container pose and its configured release offset.
