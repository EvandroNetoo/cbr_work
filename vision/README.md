# Vision

`vision` serves `/vision/analyze_scene`. Each goal selects AprilTags (bit 1),
the monochrome table surface (bit 4), HSV containers (bit 8), or a combination.
The camera is captured on demand, and each detector processes the newest
available rectified frame at its configured rate.

The table surface mask has independent inclusive HSV ranges:

- White: `table_surface_white_min_value`, `table_surface_white_max_value`,
  `table_surface_white_min_saturation`,
  `table_surface_white_max_saturation`.
- Black: `table_surface_black_min_value`, `table_surface_black_max_value`,
  `table_surface_black_min_saturation`,
  `table_surface_black_max_saturation`.

The grid then applies `table_surface_min_matching_fraction`,
`table_surface_max_overexposed_fraction`,
`table_surface_min_confirmed_frames`, and
`table_surface_min_confirmed_ratio`.

```bash
ros2 launch vision vision.launch.py
ros2 action send_goal /vision/analyze_scene interfaces/action/AnalyzeScene \
  "{requested_detectors: 8, duration: {sec: 2, nanosec: 0}, work_surface_height_m: 0.05}" --feedback
```

## Tagged cube color

The AprilTag detector reads narrow bands outside all four detected tag edges.
The bands start at the detected edges of the 32 mm tag, include its white
margin, and use small HSV crops of the rectified camera image. They come from
the detected corners; no 3D cube projection or full-frame color pass is used.
Bands with too few colored pixels or competing red and blue pixels are ignored
or marked ambiguous. The final action result requires the number of
matching frames configured by `cube_color_min_confirmed_frames`.
`color=0` means unknown; `1` is red and `2` is blue.
`color_confidence` and `color_observation_count` are carried on each AprilTag
detection. Color is independent of pose ranking, so uncertain color does not
reject a tag. The live AprilTag debug image outlines the sampled bands in
yellow. Check those bands against real camera images before relying on color
in a mission.

Acceptance thresholds are set in `config/vision.yaml`:

- `cube_color_min_band_pixels`: minimum pixels in one sampled band.
- `cube_color_min_colored_pixels`: minimum red or blue pixels in that band.
- `cube_color_min_colored_fraction`: minimum colored share of the band.
- `cube_color_min_dominance`: minimum share of one color among colored pixels
  in a band and across accepted bands.
- `cube_color_min_confirmed_frames`: minimum matching frame votes.
- `cube_color_min_vote_share`: minimum weighted share of that color among
  frame votes.

The HSV hue, saturation and value thresholds are shared with container color.

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
poses; `/table_surface/debug_image` colors white-mask pixels green and
black-mask pixels blue, with live FPS, frame/TF, observed/confirmed, and
free/blocked/unknown cell counts.
A planned release target is projected over the stored container image when
`/manipulation/container_release_target` is published.

The parameters are in `config/vision.yaml`. The placement action uses the
returned container pose and its configured release offset.
